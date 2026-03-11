import { serve, file } from "bun";
import { join, extname } from "path";
import { existsSync, readdirSync } from "fs";

const PORT = 3000;
// Root dir containing one or more tileset directories.
// Serve with: bun run server.ts <root_dir>
// Default: ../.private (relative to viewer/)
const ROOT_DIR = process.argv[2] || "../.private";
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

// Scan ROOT_DIR up to 2 levels deep for directories containing tileset.json
function discoverTilesets(rootDir: string): Array<{ label: string; path: string }> {
  const results: Array<{ label: string; path: string }> = [];
  let entries: ReturnType<typeof readdirSync>;
  try {
    entries = readdirSync(rootDir, { withFileTypes: true });
  } catch {
    return results;
  }
  for (const entry of entries) {
    if (!entry.isDirectory()) continue;
    const d1 = join(rootDir, entry.name);
    if (existsSync(join(d1, "tileset.json"))) {
      results.push({ label: entry.name, path: entry.name });
    }
    let sub: ReturnType<typeof readdirSync>;
    try {
      sub = readdirSync(d1, { withFileTypes: true });
    } catch {
      continue;
    }
    for (const e2 of sub) {
      if (!e2.isDirectory()) continue;
      const d2 = join(d1, e2.name);
      if (existsSync(join(d2, "tileset.json"))) {
        results.push({ label: `${entry.name} / ${e2.name}`, path: `${entry.name}/${e2.name}` });
      }
    }
  }
  return results;
}

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

    // API: list available tilesets
    if (pathname === "/api/tilesets") {
      const tilesets = discoverTilesets(ROOT_DIR);
      return new Response(JSON.stringify(tilesets), {
        headers: { ...headers, "Content-Type": "application/json" },
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

    // Serve tileset/asset files — everything under ROOT_DIR
    if (pathname.startsWith("/tileset/")) {
      const tilesetPath = join(ROOT_DIR, decodeURIComponent(pathname.replace("/tileset/", "")));
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
console.log(`  Root:    ${ROOT_DIR}`);
console.log(`  Switch:  http://localhost:${PORT}/?tileset=<path>\n`);
