// 3D viewer: textured surface model (GLB) or point cloud (PLY), with
// measuring tools that pick real points on the surface.
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

export type Tool = "none" | "distance" | "height" | "area" | "volume";

export interface MeasureEvent {
  tool: Tool;
  /** Picked points in the project frame (east, north, up metres). */
  points: [number, number, number][];
  finished: boolean;
}

export interface Viewer3D {
  dispose(): void;
  setPointSize(px: number): void;
  resetView(): void;
  setTool(t: Tool): void;
  clearMeasure(): void;
  finishMeasure(): void;
}

export interface ViewerOptions {
  url: string;
  kind: "glb" | "ply";
  /** glTF → project frame offset (see mesh.rs `local_offset`). */
  offset?: [number, number, number];
  onStatus: (text: string) => void;
  onMeasure: (e: MeasureEvent) => void;
}

const MARK = 0xffd23f;

export function mountViewer3D(el: HTMLElement, opt: ViewerOptions): Viewer3D {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setClearColor(0x05080c);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  el.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 100000);
  const isGlb = opt.kind === "glb";
  // glTF is y-up; the PLY is in east/north/up (z-up).
  camera.up.set(0, isGlb ? 1 : 0, isGlb ? 0 : 1);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.maxPolarAngle = Math.PI * 0.95;

  const pointsMat = new THREE.PointsMaterial({ size: 2, vertexColors: true, sizeAttenuation: false });
  let model: THREE.Object3D | null = null;
  let home: { pos: THREE.Vector3; target: THREE.Vector3 } | null = null;
  // Recentring applied on load (scene units → glTF/PLY file units).
  const centre = new THREE.Vector3();
  let span = 100;

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

  const placeCamera = (obj: THREE.Object3D) => {
    const box = new THREE.Box3().setFromObject(obj);
    box.getCenter(centre);
    obj.position.sub(centre);
    const size = new THREE.Vector3();
    box.getSize(size);
    span = Math.max(size.x, isGlb ? size.z : size.y, 1);
    const pos = isGlb ? new THREE.Vector3(0, span * 0.7, span * 0.9) : new THREE.Vector3(0, -span * 0.9, span * 0.7);
    home = { pos, target: new THREE.Vector3() };
    camera.near = span / 5000;
    camera.far = span * 20;
    camera.updateProjectionMatrix();
    controls.maxDistance = span * 6;
    resetView();
  };

  opt.onStatus("Loading 3D model…");
  if (isGlb) {
    new GLTFLoader().load(
      opt.url,
      (gltf) => {
        model = gltf.scene;
        scene.add(model);
        placeCamera(model);
        let tris = 0;
        model.traverse((o) => {
          const m = o as THREE.Mesh;
          if (m.isMesh) tris += (m.geometry.index?.count ?? 0) / 3;
        });
        opt.onStatus(`${Math.round(tris).toLocaleString()} triangles · drag to orbit, right-drag to pan, scroll to zoom`);
      },
      undefined,
      (err) => opt.onStatus(`Couldn't load the 3D model: ${String(err)}`),
    );
  } else {
    new PLYLoader().load(
      opt.url,
      (geom) => {
        model = new THREE.Points(geom, pointsMat);
        scene.add(model);
        placeCamera(model);
        opt.onStatus(`${geom.getAttribute("position").count.toLocaleString()} points · drag to orbit, right-drag to pan, scroll to zoom`);
      },
      undefined,
      (err) => opt.onStatus(`Couldn't load the 3D model: ${String(err)}`),
    );
  }

  function resetView() {
    if (!home) return;
    camera.position.copy(home.pos);
    controls.target.copy(home.target);
    controls.update();
  }

  // ───────── Measuring ─────────
  let tool: Tool = "none";
  let picks: THREE.Vector3[] = []; // scene coordinates
  let finished = false;
  const marks = new THREE.Group();
  scene.add(marks);
  const raycaster = new THREE.Raycaster();

  /** Scene point → project frame (east, north, up). */
  const toProject = (p: THREE.Vector3): [number, number, number] => {
    const f = p.clone().add(centre); // file coordinates
    if (isGlb) {
      const o = opt.offset ?? [0, 0, 0];
      return [f.x + o[0], -f.z + o[1], f.y + o[2]];
    }
    return [f.x, f.y, f.z];
  };

  const redraw = () => {
    marks.clear();
    if (!picks.length) return;
    const r = span / 400;
    const sphere = new THREE.SphereGeometry(r, 12, 8);
    const mat = new THREE.MeshBasicMaterial({ color: MARK, depthTest: false });
    for (const p of picks) {
      const s = new THREE.Mesh(sphere, mat);
      s.position.copy(p);
      s.renderOrder = 10;
      marks.add(s);
    }
    const lineMat = new THREE.LineBasicMaterial({ color: MARK, depthTest: false });
    const closed = (tool === "area" || tool === "volume") && (finished || picks.length > 2);
    const pts = closed ? [...picks, picks[0]] : picks;
    if (pts.length > 1) {
      const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), lineMat);
      l.renderOrder = 10;
      marks.add(l);
    }
    if (tool === "height" && picks.length === 2) {
      // Show the vertical drop as a plumb line from the higher point.
      const up = new THREE.Vector3(...(isGlb ? [0, 1, 0] : [0, 0, 1]));
      const [a, b] = picks;
      const ha = a.dot(up);
      const hb = b.dot(up);
      const top = ha > hb ? a : b;
      const low = ha > hb ? b : a;
      const foot = top.clone().sub(up.clone().multiplyScalar(top.dot(up) - low.dot(up)));
      const plumb = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([top, foot, low]),
        new THREE.LineDashedMaterial({ color: 0x36e0c0, dashSize: span / 300, gapSize: span / 400, depthTest: false }),
      );
      plumb.computeLineDistances();
      plumb.renderOrder = 10;
      marks.add(plumb);
    }
  };

  const emit = () => opt.onMeasure({ tool, points: picks.map(toProject), finished });

  let downAt: { x: number; y: number } | null = null;
  const onDown = (e: PointerEvent) => {
    downAt = { x: e.clientX, y: e.clientY };
  };
  const onUp = (e: PointerEvent) => {
    if (tool === "none" || !model || e.button !== 0 || !downAt) return;
    // A drag is an orbit, not a pick.
    if (Math.hypot(e.clientX - downAt.x, e.clientY - downAt.y) > 4) return;
    const rect = renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
    raycaster.setFromCamera(ndc, camera);
    raycaster.params.Points = { threshold: span / 500 };
    const hit = raycaster.intersectObject(model, true)[0];
    if (!hit) return;
    if (finished) {
      picks = [];
      finished = false;
    }
    // The two clicks of a double-click hit the same spot: keep one.
    const last = picks[picks.length - 1];
    if (last && last.distanceTo(hit.point) < span / 2000) return;
    picks.push(hit.point.clone());
    const limit = tool === "height" ? 2 : Infinity;
    if (picks.length >= limit) finished = true;
    redraw();
    emit();
  };
  const onDbl = () => {
    if ((tool === "area" || tool === "volume" || tool === "distance") && picks.length >= 2) {
      finished = true;
      redraw();
      emit();
    }
  };
  renderer.domElement.addEventListener("pointerdown", onDown);
  renderer.domElement.addEventListener("pointerup", onUp);
  renderer.domElement.addEventListener("dblclick", onDbl);

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
      scene.traverse((o) => {
        const m = o as THREE.Mesh;
        m.geometry?.dispose?.();
        const mats = Array.isArray(m.material) ? m.material : m.material ? [m.material] : [];
        mats.forEach((mt: any) => {
          mt.map?.dispose?.();
          mt.dispose?.();
        });
      });
      renderer.dispose();
      renderer.domElement.remove();
    },
    setPointSize(px: number) {
      pointsMat.size = px;
    },
    resetView,
    setTool(t: Tool) {
      tool = t;
      picks = [];
      finished = false;
      redraw();
      renderer.domElement.style.cursor = t === "none" ? "" : "crosshair";
      emit();
    },
    clearMeasure() {
      picks = [];
      finished = false;
      redraw();
      emit();
    },
    finishMeasure() {
      onDbl();
    },
  };
}
