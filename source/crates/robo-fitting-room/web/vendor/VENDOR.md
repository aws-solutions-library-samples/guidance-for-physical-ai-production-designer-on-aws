# Vendored runtime dependencies

These files are committed so the editor page loads with **no CDN or network
fetch at runtime**: `serve` embeds `web/` into the binary (`include_dir!`), so
`/vendor/...` is served by the binary itself, like `/style.css`. They are fetched
once at authoring time and pinned here. Do not add a script tag that loads from
another host, or an `import` from a URL, to `web/`: vendor the file here instead.

| Path | Package | Version | License | sha256 of the file |
|------|---------|---------|---------|--------------------|
| `js-yaml/js-yaml.min.js` | [js-yaml](https://github.com/nodeca/js-yaml) (`dist/js-yaml.min.js`) | 4.1.1 | MIT | `0de3dec92d20eab9e0b46a5d928cd45ec025d73e348ddf458dbfb01da00cb473` |
| `js-yaml/LICENSE` | js-yaml (`LICENSE`) | 4.1.1 | MIT | `a07bc24468b9654ce76a547d47a2db282d07733b715db4c73a98bd63961f9550` |

`web/index.html` loads `/vendor/js-yaml/js-yaml.min.js` as a classic script; it
defines the global `jsyaml` that `web/js/app.js` uses.

Source: the npm registry tarball `https://registry.npmjs.org/js-yaml/-/js-yaml-4.1.1.tgz`,
checked against the registry's `dist.integrity`
(`sha512-qQKT4zQxXl8lLwBtHMWwaTcGfFOZviOJet3Oy/xmGk2gZH677CJM9EvtfdSkgWcATZhj/55JZ0rmy3myCT5lsA==`;
the tarball's sha256 is `948181607d4fb62dde7b9e04a62d89b20e62cd0c5659ceb711bc31768d00436f`)
before the two files were copied out of its `package/` folder unchanged.

To refresh: download `https://registry.npmjs.org/js-yaml/<version>` (the
metadata) and the tarball its `dist.tarball` names into an empty directory,
check the tarball's sha512 against `dist.integrity`, copy `package/dist/js-yaml.min.js`
and `package/LICENSE` here, and update the versions and checksums above (and the
js-yaml row of the public export's THIRD-PARTY-NOTICES.md).
