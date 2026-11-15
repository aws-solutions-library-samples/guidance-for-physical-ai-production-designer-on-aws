// Bundles @eclipse-zenoh/zenoh-ts into one browser ES module (web/dist/vendor/zenoh-ts.js), which
// web/index.html maps the bare "@eclipse-zenoh/zenoh-ts" specifier of js/zenoh_bridge.js to.
//
// zenoh-ts imports its key-expression wasm the bundler way (`import * as wasm from "x.wasm"`),
// which browsers can't do. The plugin below turns each such import into a module that inlines the
// bytes and instantiates them with the imports the wasm declares, re-exporting its exports.
//
//   node web/bundle-zenoh-ts.mjs <node_modules dir> <out file>
import { readFile } from "node:fs/promises";
import path from "node:path";
import { createRequire } from "node:module";

const [nodeModules, outfile] = process.argv.slice(2);
if (!nodeModules || !outfile) {
  console.error("usage: node bundle-zenoh-ts.mjs <node_modules dir> <out file>");
  process.exit(2);
}
const require = createRequire(path.join(path.resolve(nodeModules), "noop.js"));
const esbuild = require("esbuild");

const wasmEsm = {
  name: "wasm-esm",
  setup(build) {
    build.onResolve({ filter: /\.wasm$/ }, (args) =>
      args.namespace === "wasm-stub"
        ? { path: args.path, namespace: "wasm-binary" }
        : { path: path.resolve(args.resolveDir, args.path), namespace: "wasm-stub" },
    );
    build.onLoad({ filter: /.*/, namespace: "wasm-stub" }, async (args) => {
      const module = await WebAssembly.compile(await readFile(args.path));
      const importModules = [...new Set(WebAssembly.Module.imports(module).map((i) => i.module))];
      const exportNames = WebAssembly.Module.exports(module).map((e) => e.name);
      const lines = importModules.map((m, i) => `import * as imports${i} from ${JSON.stringify(m)};`);
      lines.push(`import bytes from ${JSON.stringify(args.path)};`);
      const importObject = importModules.map((m, i) => `${JSON.stringify(m)}: imports${i}`).join(", ");
      lines.push(`const { instance } = await WebAssembly.instantiate(bytes, { ${importObject} });`);
      for (const name of exportNames) {
        lines.push(`export const ${name} = instance.exports[${JSON.stringify(name)}];`);
      }
      return { contents: lines.join("\n"), resolveDir: path.dirname(args.path) };
    });
    build.onLoad({ filter: /.*/, namespace: "wasm-binary" }, async (args) => ({
      contents: await readFile(args.path),
      loader: "binary",
    }));
  },
};

await esbuild.build({
  stdin: {
    contents: 'export * from "@eclipse-zenoh/zenoh-ts";',
    resolveDir: path.resolve(nodeModules, ".."),
  },
  nodePaths: [path.resolve(nodeModules)],
  bundle: true,
  format: "esm",
  platform: "browser",
  target: "es2022", // top-level await (the wasm instantiation above)
  minify: true,
  outfile,
  plugins: [wasmEsm],
});
console.log(`bundled zenoh-ts -> ${outfile}`);
