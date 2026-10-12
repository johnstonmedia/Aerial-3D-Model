// Point-cloud viewer: orbit, pan, zoom around the reconstructed site.
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";

export interface Viewer3D {
  dispose(): void;
  setPointSize(px: number): void;
  resetView(): void;
}

/** Loads `url` (binary PLY, ENU metres, z up) into `el`. */
export function mountViewer3D(el: HTMLElement, url: string, onStatus: (text: string) => void): Viewer3D {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setClearColor(0x05080c);
  el.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 100000);
  camera.up.set(0, 0, 1); // the cloud is east/north/up
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.screenSpacePanning = false;
  controls.maxPolarAngle = Math.PI * 0.95;

  const material = new THREE.PointsMaterial({ size: 2, vertexColors: true, sizeAttenuation: false });
  let points: THREE.Points | null = null;
  let home: { pos: THREE.Vector3; target: THREE.Vector3 } | null = null;

  const resize = () => {
    const w = el.clientWidth || 1;
    const h = el.clientHeight || 1;
    renderer.setSize(w, h, false);
    renderer.domElement.style.width = "100%";
    renderer.domElement.style.height = "100%";
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  const ro = new ResizeObserver(resize);
  ro.observe(el);
  resize();

  onStatus("Loading point cloud…");
  new PLYLoader().load(
    url,
    (geom) => {
      // Recentre for float precision and a sensible orbit pivot.
      geom.computeBoundingBox();
      const bb = geom.boundingBox!;
      const centre = new THREE.Vector3();
      bb.getCenter(centre);
      geom.translate(-centre.x, -centre.y, -centre.z);
      points = new THREE.Points(geom, material);
      scene.add(points);
      const size = new THREE.Vector3();
      bb.getSize(size);
      const span = Math.max(size.x, size.y, 1);
      const target = new THREE.Vector3(0, 0, 0);
      const pos = new THREE.Vector3(0, -span * 0.9, span * 0.7);
      home = { pos, target };
      camera.near = span / 2000;
      camera.far = span * 20;
      camera.updateProjectionMatrix();
      controls.maxDistance = span * 6;
      resetView();
      const n = geom.getAttribute("position").count;
      onStatus(`${n.toLocaleString()} points · drag to orbit, right-drag to pan, scroll to zoom`);
    },
    undefined,
    (err) => onStatus(`Couldn't load the 3D model: ${String(err)}`),
  );

  function resetView() {
    if (!home) return;
    camera.position.copy(home.pos);
    controls.target.copy(home.target);
    controls.update();
  }

  let raf = 0;
  const tick = () => {
    raf = requestAnimationFrame(tick);
    controls.update();
    renderer.render(scene, camera);
  };
  tick();

  return {
    dispose() {
      cancelAnimationFrame(raf);
      ro.disconnect();
      controls.dispose();
      points?.geometry.dispose();
      material.dispose();
      renderer.dispose();
      renderer.domElement.remove();
    },
    setPointSize(px: number) {
      material.size = px;
    },
    resetView,
  };
}
