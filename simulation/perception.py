import base64
import time
import math
import numpy as np
import cv2
import pybullet as pb
from typing import Dict, Any, List, Optional
from simulation.robot_model import LIFT_CARRIAGE_LINK

class PerceptionSystem:
    def __init__(self, world, lock):
        self.world = world
        self.lock = lock
        self.width = 320
        self.height = 240
        self.fov = 60.0
        self.near = 0.1
        self.far = 10.0
        self.projection_matrix = pb.computeProjectionMatrixFOV(
            fov=self.fov, aspect=self.width / self.height, nearVal=self.near, farVal=self.far
        )
        self.enabled = True

    def capture_and_process(self) -> Optional[Dict[str, Any]]:
        if not self.enabled or not self.world.is_initialized:
            return None

        t0 = time.perf_counter()

        # 1. Thread-safe capture
        with self.lock:
            # Get lift link state
            lift_link_idx = self.world.model.links.get(LIFT_CARRIAGE_LINK)
            if lift_link_idx is None:
                return None
            
            link_state = pb.getLinkState(self.world.robot_id, lift_link_idx, computeForwardKinematics=True, physicsClientId=self.world.client_id)
            link_pos = link_state[0] # [x, y, z]
            link_orn = link_state[1] # quaternion

            # Calculate camera position (slightly forward from lift mast)
            rot_matrix = pb.getMatrixFromQuaternion(link_orn)
            rot_matrix = np.array(rot_matrix).reshape(3, 3)
            
            # Mount camera slightly in front of the carriage, looking forward
            camera_offset = np.array([0.15, 0.0, 0.0]) 
            camera_pos = np.array(link_pos) + rot_matrix.dot(camera_offset)
            
            target_offset = np.array([1.0, 0.0, 0.0])
            target_pos = camera_pos + rot_matrix.dot(target_offset)
            
            up_vector = rot_matrix.dot(np.array([0.0, 0.0, 1.0]))

            view_matrix = pb.computeViewMatrix(
                cameraEyePosition=camera_pos.tolist(),
                cameraTargetPosition=target_pos.tolist(),
                cameraUpVector=up_vector.tolist()
            )

            # Render
            _, _, rgba, depth, seg = pb.getCameraImage(
                width=self.width,
                height=self.height,
                viewMatrix=view_matrix,
                projectionMatrix=self.projection_matrix,
                renderer=pb.ER_BULLET_HARDWARE_OPENGL,
                physicsClientId=self.world.client_id
            )

            # Prepare reverse map for body IDs to get ground truth
            id_to_name = {v: k for k, v in self.world.static_body_ids.items()}
            id_to_name.update({v: k for k, v in self.world.container_ids.items()})

            gt_positions = {}
            for body_id in id_to_name.keys():
                pos, _ = pb.getBasePositionAndOrientation(body_id, physicsClientId=self.world.client_id)
                gt_positions[body_id] = np.array(pos)

        # 2. Offline processing (Lock released, safe for physics thread)
        rgba_np = np.array(rgba, dtype=np.uint8).reshape((self.height, self.width, 4))
        rgb = cv2.cvtColor(rgba_np, cv2.COLOR_RGBA2RGB)
        depth_np = np.array(depth, dtype=np.float32).reshape((self.height, self.width))
        seg_np = np.array(seg, dtype=np.int32).reshape((self.height, self.width))

        # True depth in meters
        true_depth = self.far * self.near / (self.far - (self.far - self.near) * depth_np)

        detected_objects = []

        # Projection matrix inverse for deprojection
        proj_mat = np.array(self.projection_matrix).reshape(4,4).T
        inv_view = np.linalg.inv(np.array(view_matrix).reshape(4,4).T)

        unique_ids = np.unique(seg_np)
        for obj_id in unique_ids:
            body_id = obj_id & ((1 << 24) - 1)
            if body_id == self.world.floor_id or body_id == self.world.robot_id or body_id < 0:
                continue

            name = id_to_name.get(body_id, "Unknown")
            if name == "Unknown":
                continue
                
            category = "Container" if "TOTE" in name else "Rack" if "Rack" in name or "Shelf" in name or "Stand" in name else "Obstacle"
            
            mask = np.uint8(seg_np == obj_id) * 255
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                continue
                
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) < 10:
                continue
                
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Use all valid depths in the bounding box/mask to estimate object center
            obj_depths = true_depth[mask == 255]
            valid_depths = obj_depths[obj_depths < 0.99 * self.far]
            if len(valid_depths) == 0:
                continue
                
            avg_d = np.median(valid_depths)
            
            # Instead of surface, let's roughly add half bounding box extent for depth center 
            # Or simpler: just compare surface to GT center.
            # We'll calculate the 3D position of the center of the bounding box.
            cx, cy = x + w / 2.0, y + h / 2.0
            ndc_x = (2.0 * cx / self.width) - 1.0
            ndc_y = 1.0 - (2.0 * cy / self.height)
            
            x_cam = avg_d * ndc_x / proj_mat[0,0]
            y_cam = avg_d * ndc_y / proj_mat[1,1]
            z_cam = -avg_d
            point_cam = np.array([x_cam, y_cam, z_cam, 1.0])
            
            point_world = inv_view.dot(point_cam)
            point_world = point_world[:3] / point_world[3]
            
            gt_pos = gt_positions.get(body_id)
            aabb_boundary_distance = None
            if gt_pos is not None:
                # To get AABB boundary distance, we find the AABB of the object and find distance from point_world to AABB
                aabb_min, aabb_max = pb.getAABB(body_id, physicsClientId=self.world.client_id)
                aabb_min = np.array(aabb_min)
                aabb_max = np.array(aabb_max)
                # Closest point on AABB to point_world
                closest_point = np.clip(point_world, aabb_min, aabb_max)
                aabb_boundary_distance = float(np.linalg.norm(point_world - closest_point) * 100.0) # AABB boundary distance in cm
                
            detected_objects.append({
                "category": category,
                "instance_id": name,
                "bbox": [int(x), int(y), int(w), int(h)],
                "distance": float(avg_d),
                "position": {"x": float(point_world[0]), "y": float(point_world[1]), "z": float(point_world[2])},
                "aabb_boundary_distance": aabb_boundary_distance
            })
            
            # Draw on RGB preview
            cv2.rectangle(rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(rgb, f"{name} ({avg_d:.1f}m)", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        t1 = time.perf_counter()
        
        _, buffer = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 70])
        rgb_base64 = base64.b64encode(buffer).decode('utf-8')

        return {
            "timestamp": time.time(),
            "latency_ms": (t1 - t0) * 1000.0,
            "rgb_base64": rgb_base64,
            "objects": detected_objects
        }
