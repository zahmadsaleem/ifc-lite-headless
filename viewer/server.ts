import { serve, file } from "bun";
import { join, extname } from "path";
import { existsSync } from "fs";

const PORT = 3000;
const TILESET_DIR = process.argv[2] || "../.private/testdata/tileset_output";
const NODE_MODULES = join(import.meta.dir, "node_modules");

const MIME_TYPES: Record<string, string> = {
  ".html": "text/html",
  ".js": "application/javascript",
  ".mjs": "application/javascript",
  ".css": "text/css",
  ".json": "application/json",
  ".glb": "model/gltf-binary",
  ".png": "image/png",
  ".svg": "image/svg+xml",
  ".wasm": "application/wasm",
  ".map": "application/json",
};

const headers = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Methods": "GET, OPTIONS",
  "Access-Control-Allow-Headers": "*",
};

serve({
  port: PORT,
  async fetch(req) {
    const url = new URL(req.url);
    let pathname = url.pathname;

    if (req.method === "OPTIONS") {
      return new Response(null, { headers });
    }

    // Serve index at root
    if (pathname === "/" || pathname === "/index.html") {
      return new Response(file(join(import.meta.dir, "index.html")), {
        headers: { ...headers, "Content-Type": "text/html" },
      });
    }

    // Serve node_modules for ESM imports
    if (pathname.startsWith("/node_modules/")) {
      const filePath = join(import.meta.dir, pathname);
      if (existsSync(filePath)) {
        const ext = extname(filePath);
        const mime = MIME_TYPES[ext] || "application/javascript";
        return new Response(file(filePath), {
          headers: { ...headers, "Content-Type": mime },
        });
      }
      return new Response("Not found", { status: 404 });
    }

    // Serve tileset files
    if (pathname.startsWith("/tileset/")) {
      const tilesetPath = join(TILESET_DIR, pathname.replace("/tileset/", ""));
      if (existsSync(tilesetPath)) {
        const ext = extname(tilesetPath);
        const mime = MIME_TYPES[ext] || "application/octet-stream";
        return new Response(file(tilesetPath), {
          headers: { ...headers, "Content-Type": mime },
        });
      }
      return new Response("Not found", { status: 404 });
    }

    return new Response("Not found", { status: 404 });
  },
});

console.log(`\n  Viewer:  http://localhost:${PORT}`);
console.log(`  Tileset: ${TILESET_DIR}\n`);
