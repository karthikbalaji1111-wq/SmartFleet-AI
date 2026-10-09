import * as THREE from 'three';
import type { Vec3 } from '../api/types';

/** URDF rpy (fixed-axis roll-pitch-yaw) == Three.js Euler with order 'ZYX'. */
export function rpyEuler(rpy: Vec3): THREE.Euler {
  return new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX');
}

/** Release GPU resources of everything under ``root``. */
export function disposeObject(root: THREE.Object3D): void {
  root.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (mesh.geometry) mesh.geometry.dispose();
    const material = (mesh as { material?: THREE.Material | THREE.Material[] }).material;
    const materials = Array.isArray(material) ? material : material ? [material] : [];
    for (const m of materials) {
      for (const value of Object.values(m)) {
        if (value instanceof THREE.Texture) value.dispose();
      }
      m.dispose();
    }
  });
}

/** Text rendered to a canvas texture (used for floor markings and labels). */
export function textTexture(
  text: string,
  { color = '#ffffff', background = 'rgba(0,0,0,0)', fontPx = 64, weight = 700 } = {},
): { texture: THREE.CanvasTexture; aspect: number } {
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d')!;
  const font = `${weight} ${fontPx}px system-ui, "Segoe UI", sans-serif`;
  ctx.font = font;
  const pad = fontPx * 0.35;
  canvas.width = Math.ceil(ctx.measureText(text).width + pad * 2);
  canvas.height = Math.ceil(fontPx * 1.4);
  ctx.font = font;
  ctx.fillStyle = background;
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = color;
  ctx.textBaseline = 'middle';
  ctx.fillText(text, pad, canvas.height / 2);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 4;
  return { texture, aspect: canvas.width / canvas.height };
}

/** Camera-facing label sprite of a given world height (metres). */
export function labelSprite(text: string, height: number, color = '#e6edf3'): THREE.Sprite {
  const { texture, aspect } = textTexture(text, { color, background: 'rgba(13,17,23,0.72)', fontPx: 56 });
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthWrite: false }));
  sprite.scale.set(height * aspect, height, 1);
  return sprite;
}
