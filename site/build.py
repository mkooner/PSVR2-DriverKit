#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Build the PSVR2-DriverKit website into site/_site (Python standard library only).

Each page is site/pages/<name>.html: a JSON front matter block in an HTML comment, then the page's HTML body:

    <!--
    {"path": "features/", "title": "...", "description": "...", "breadcrumb": "Features"}
    -->
    <h1>...</h1> ...

Inside pages and the template, {{root}} is the site's root path (for example /PSVR2-DriverKit/), {{repo}} the
repository URL. The build writes each page to <path>index.html, plus 404.html, sitemap.xml, robots.txt, .nojekyll,
CNAME (when site.json or --cname names a custom domain) and the assets.

    python3 site/build.py [--base-url URL] [--cname DOMAIN] [--out DIR]
"""
import argparse
import datetime
import html
import json
import os
import re
import shutil
import subprocess
import sys
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)


def load_pages():
    pages = []
    for name in sorted(os.listdir(os.path.join(HERE, "pages"))):
        if not name.endswith(".html"):
            continue
        with open(os.path.join(HERE, "pages", name), encoding="utf-8") as f:
            text = f.read()
        m = re.match(r"\s*<!--\s*(\{.*?\})\s*-->\s*", text, re.S)
        if not m:
            sys.exit(f"{name}: no JSON front matter")
        meta = json.loads(m.group(1))
        meta["body"] = text[m.end():]
        meta["source"] = os.path.join("site", "pages", name)
        if "path" not in meta:
            sys.exit(f"{name}: front matter needs 'path' (\"\" for the home page)")
        for key in ("title", "description"):
            if not meta.get(key):
                sys.exit(f"{name}: front matter needs '{key}'")
        pages.append(meta)
    return pages


def last_modified(paths):
    """The newest git commit date of these files, else today (UTC)."""
    try:
        out = subprocess.run(["git", "-C", REPO_ROOT, "log", "-1", "--format=%cs", "--", *paths],
                             capture_output=True, text=True, check=True).stdout.strip()
        if out:
            return out
    except (OSError, subprocess.CalledProcessError):
        pass
    return datetime.datetime.now(datetime.timezone.utc).date().isoformat()


def jsonld_for(page, cfg, canonical, home):
    graph = []
    if page["path"] == "":
        app = {
            "@type": "SoftwareApplication",
            "@id": home + "#software",
            "name": cfg["name"],
            "description": page["description"],
            "url": home,
            "applicationCategory": "DriverApplication",
            "operatingSystem": "Linux",
            "softwareRequirements": "Sony PlayStation VR2 headset with Sony's PS VR2 PC adapter; Steam and SteamVR for SteamVR games",
            "license": cfg["license_url"],
            "isAccessibleForFree": True,
            "downloadUrl": cfg["repo"] + "/releases/latest",
            "codeRepository": cfg["repo"],
            "image": home + cfg["og_image"],
        }
        code = {
            "@type": "SoftwareSourceCode",
            "@id": home + "#source",
            "name": cfg["name"],
            "description": "Source code of PSVR2-DriverKit: Monado fork with the PS VR2 and Sense drivers, the native SteamVR driver and the DriverKit app.",
            "codeRepository": cfg["repo"],
            "programmingLanguage": ["C", "C++", "Rust", "Shell", "Python"],
            "runtimePlatform": "Linux",
            "license": cfg["license_url"],
            "targetProduct": {"@id": home + "#software"},
        }
        site = {"@type": "WebSite", "@id": home + "#website", "name": cfg["name"], "url": home}
        graph += [site, app, code]
    else:
        crumbs = [{"@type": "ListItem", "position": 1, "name": "Home", "item": home},
                  {"@type": "ListItem", "position": 2, "name": page.get("breadcrumb", page["title"]),
                   "item": canonical}]
        graph.append({"@type": "BreadcrumbList", "itemListElement": crumbs})
        graph.append({"@type": "WebPage", "name": page["title"], "description": page["description"],
                      "url": canonical, "isPartOf": {"@id": home + "#website"}})
    return json.dumps({"@context": "https://schema.org", "@graph": graph}, indent=1, ensure_ascii=False)


def render(template, values):
    def sub(m):
        key = m.group(1)
        if key not in values:
            sys.exit(f"template placeholder {{{{{key}}}}} has no value")
        return values[key]
    return re.sub(r"\{\{(\w+)\}\}", sub, template)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base-url", help="site URL ending in / (default: site.json, or https://CNAME/ with a CNAME)")
    ap.add_argument("--cname", help="custom domain to write as CNAME (default: site.json's cname)")
    ap.add_argument("--out", default=os.path.join(HERE, "_site"))
    args = ap.parse_args()

    with open(os.path.join(HERE, "site.json"), encoding="utf-8") as f:
        cfg = json.load(f)
    cname = args.cname if args.cname is not None else cfg.get("cname", "")
    base = args.base_url or (f"https://{cname}/" if cname else cfg["base_url"])
    if not base.endswith("/"):
        base += "/"
    root = urlparse(base).path or "/"
    with open(os.path.join(HERE, "templates", "base.html"), encoding="utf-8") as f:
        template = f.read()

    pages = load_pages()
    paths = [p["path"] for p in pages]
    if len(set(paths)) != len(paths):
        sys.exit("two pages share a path")

    out = args.out
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    shutil.copytree(os.path.join(HERE, "assets"), os.path.join(out, "assets"))
    # The social preview (rendered from site/assets/social-preview.svg) doubles as the Open Graph image.
    shutil.copy(os.path.join(REPO_ROOT, "docs", "images", "social-preview.png"), os.path.join(out, "assets", "og.png"))
    shutil.copy(os.path.join(REPO_ROOT, "docs", "images", "driverkit-compile.png"),
                os.path.join(out, "assets", "driverkit-compile.png"))

    verification = ""
    if cfg.get("google_site_verification"):
        verification += f'<meta name="google-site-verification" content="{html.escape(cfg["google_site_verification"])}">\n'
    if cfg.get("bing_site_verification"):
        verification += f'<meta name="msvalidate.01" content="{html.escape(cfg["bing_site_verification"])}">\n'

    sitemap = []
    for page in pages:
        canonical = base + page["path"]
        nav = []
        for label, href in cfg["nav"]:
            current = ' aria-current="page"' if href == page["path"] else ""
            nav.append(f'        <li><a href="{root}{href}"{current}>{html.escape(label)}</a></li>')
        crumbs = ""
        if page["path"] not in ("", "404/"):
            crumbs = (f'<nav class="crumbs" aria-label="Breadcrumb"><a href="{root}">Home</a> <span aria-hidden="true">›</span> '
                      f'<span>{html.escape(page.get("breadcrumb", page["title"]))}</span></nav>\n')
        is404 = page["path"] == "404/"
        values = {
            "title": html.escape(page["title"]),
            "description": html.escape(page["description"]),
            "canonical": html.escape(canonical),
            "robots": "noindex" if is404 else "index, follow",
            "verification": verification,
            "og_type": "website",
            "site_name": html.escape(cfg["name"]),
            "og_image": html.escape(base + cfg["og_image"]),
            "og_image_alt": html.escape(cfg["og_image_alt"]),
            "root": root,
            "repo": cfg["repo"],
            "license_url": cfg["license_url"],
            "license_name": html.escape(cfg["license_name"]),
            "nav": "\n".join(nav),
            "breadcrumbs": crumbs,
            "jsonld": jsonld_for(page, cfg, canonical, base).replace("</", "<\\/"),
            "content": render(page["body"], {"root": root, "repo": cfg["repo"]}),
        }
        doc = render(template, values)
        if is404:
            target = os.path.join(out, "404.html")
        else:
            target = os.path.join(out, page["path"], "index.html")
            sitemap.append((canonical, last_modified([page["source"], "site/templates/base.html"])))
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "w", encoding="utf-8") as f:
            f.write(doc)

    with open(os.path.join(out, "sitemap.xml"), "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n'
                '<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">\n')
        for loc, lastmod in sitemap:
            f.write(f"  <url><loc>{html.escape(loc)}</loc><lastmod>{lastmod}</lastmod></url>\n")
        f.write("</urlset>\n")
    with open(os.path.join(out, "robots.txt"), "w", encoding="utf-8") as f:
        f.write(f"User-agent: *\nAllow: /\n\nSitemap: {base}sitemap.xml\n")
    open(os.path.join(out, ".nojekyll"), "w").close()
    if cname:
        with open(os.path.join(out, "CNAME"), "w") as f:
            f.write(cname + "\n")
    print(f"built {len(pages)} pages into {out} for {base}")


if __name__ == "__main__":
    main()
