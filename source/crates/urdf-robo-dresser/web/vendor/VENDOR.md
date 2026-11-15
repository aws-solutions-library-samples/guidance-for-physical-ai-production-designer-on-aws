# Vendored runtime dependencies (NFR-014)

These files are committed so the SPA renders with **no CDN / network fetch at
runtime** (FR-052 "without external dependencies"). They are fetched once at
authoring time and pinned here. Do not add a script tag that loads from another
host, or an `import` from a URL, to `web/` — resolve everything through the import
map in `web/index.html`.

| Path | Package (file in the npm tarball's `package/`) | Version | License | sha256 of the file |
|------|---------|---------|---------|--------------------|
| `three/three.module.js` | [three](https://github.com/mrdoob/three.js) (`build/three.module.js`) | r0.169.0 | MIT | `0a3368c165eea773490aec7b77c22de70e3eac288503409256fdbf4d12578416` |
| `three/addons/controls/OrbitControls.js` | three examples/jsm | r0.169.0 | MIT | `80efaadea4f8a636a65fb0bd08bfef62f3d93a0bb94e2e7500f23176c5c07f4e` |
| `three/addons/loaders/OBJLoader.js` | three examples/jsm | r0.169.0 | MIT | `d7623bf8959fdbb95302730d5fe1510484b6d16ed975c90f43312ee57546c540` |
| `three/addons/loaders/MTLLoader.js` | three examples/jsm | r0.169.0 | MIT | `6e0819c7be4ed1d30205eb3003b149ccbc6e018602fe460d297a7393d4f712ef` |
| `three/addons/loaders/STLLoader.js` | three examples/jsm | r0.169.0 | MIT | `a0a83c88b269c94e25b690fae770d350c4728c81853195186976be7af0f8a3b3` |
| `three/addons/loaders/ColladaLoader.js` | three examples/jsm | r0.169.0 | MIT | `f306e36d44e80cad00db4e978f9e19dd720c849bd979569dfa95dc1e4348bf1a` |
| `three/addons/loaders/TGALoader.js` | three examples/jsm (ColladaLoader dep) | r0.169.0 | MIT | `ce938459ed99d5234f9665270cca8552464d0d1703a094e49760631e6b14c7a3` |
| `three/addons/helpers/ViewHelper.js` | three examples/jsm (corner axis gizmo) | r0.169.0 | MIT | `2898eeb913e3f125e185b2e9e0e91c0cc278b676b3bad4cfb0afbe6e8a5446c5` |
| `three/LICENSE` | three (`LICENSE`) | r0.169.0 | MIT | `4c40a1ef62450b857c3b2aaf294936304cd552d965fbcd9d32d4c5bcf4ba4454` |
| `urdf-loader/URDFLoader.js` | [urdf-loader](https://github.com/gkjohnson/urdf-loaders) (`src/URDFLoader.js`) | 0.12.6 | Apache-2.0 | `08d63008ab689ba1f0dbc7a1936b4cc7e7a45e15036f6927028226f2bf95c0e6` |
| `urdf-loader/URDFClasses.js` | urdf-loader (`src/URDFClasses.js`) | 0.12.6 | Apache-2.0 | `d5112785611d360b501e824a4811252a57a0a44b761138daa6a35041b06bd569` |
| `urdf-loader/URDFDragControls.js` | urdf-loader (`src/URDFDragControls.js`, pointer joint dragging) | 0.12.6 | Apache-2.0 | `b3bb8de9f432c0eef40af6cb789867c3ee861ff9ffc483b526ea2d626a5e30fd` |

Checked on 2026-11-11: every file above is byte-identical to the one in the npm
registry tarball, and each tarball matched the registry's `dist.integrity`:

| Tarball | `dist.integrity` | sha256 of the tarball |
|---------|------------------|-----------------------|
| `https://registry.npmjs.org/three/-/three-0.169.0.tgz` | `sha512-Ed906MA3dR4TS5riErd4QBsRGPcx+HBDX2O5yYE5GqJeFQTPU+M56Va/f/Oph9X7uZo3W3o4l2ZhBZ6f6qUv0w==` | `c86d0937570fb425d981e156ddf919c43dbc45d4e91fd7c8dd0de56723b3ec71` |
| `https://registry.npmjs.org/urdf-loader/-/urdf-loader-0.12.6.tgz` | `sha512-EwpgOCPe6Tep2+MXoo/r13keHaKQXMcM+4s9+jX0NRxNS/QSNuP5JPdk5AIgWEoEB43AkEj9Vk+Nr53NkXgSbA==` | `8bb10206c2df8521551b1f0a8d27806b11c42df401208665d10b2c392d2f18f0` |

The urdf-loader package ships no `LICENSE` file: its licence is the
`"license": "Apache-2.0"` of its `package.json` (and the upstream repository's
`LICENSE`).

Import-map specifiers (see `web/index.html`):
- `three` → `three/three.module.js`
- `three/addons/` → `three/addons/` (this crate's own imports)
- `three/examples/jsm/` → `three/addons/` (urdf-loader imports STL/Collada loaders under this path)
- `urdf-loader` → `urdf-loader/URDFLoader.js`

To refresh: download the package's npm registry tarball
(`https://registry.npmjs.org/<pkg>/-/<pkg>-<ver>.tgz`) into an empty directory,
check its sha512 against the `dist.integrity` of `https://registry.npmjs.org/<pkg>/<ver>`,
copy each file from its `package/` folder (the paths above) at the new version,
update the versions and checksums here, then confirm every
`import ... from '...'` still resolves to one of the specifiers listed here
(grep for `http`/`https` — there must be none).
