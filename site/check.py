#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Check the built website (site/_site) and, with --markdown, the repository's Markdown docs.

Website checks: every internal link and image resolves to a built file; each page has a unique title and meta
description (description 70-170 characters), a canonical URL equal to its own URL, Open Graph and Twitter tags,
html lang, a viewport, exactly one h1 and no skipped heading levels, alt text on every image, parseable JSON-LD,
balanced HTML tags; sitemap.xml parses and lists exactly the indexable pages; robots.txt names the sitemap.

--markdown: relative links (and #anchors, GitHub style) in README.md, CONTRIBUTING.md, SECURITY.md,
CODE_OF_CONDUCT.md and docs/*.md resolve.
--external: also request every external http(s) link once (slow; needs network).

Exit status 1 on any problem. Python standard library only.
"""
import argparse
import glob
import json
import os
import re
import sys
import urllib.request
import xml.etree.ElementTree as ET
from html.parser import HTMLParser
from urllib.parse import urljoin, urlparse, unquote

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
VOID = {"area", "base", "br", "col", "embed", "hr", "img", "input", "link", "meta", "source", "track", "wbr"}

problems = []


def problem(where, msg):
    problems.append(f"{where}: {msg}")


class Page(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.stack, self.links, self.imgs, self.headings, self.meta, self.jsonld = [], [], [], [], {}, []
        self.title, self.lang, self.canonical, self.ids = None, None, None, set()
        self._in = None
        self._buf = ""

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if tag not in VOID:
            self.stack.append((tag, self.getpos()[0]))
        if "id" in a:
            self.ids.add(a["id"])
        if tag == "html":
            self.lang = a.get("lang")
        elif tag == "a" and "href" in a:
            self.links.append(a["href"])
        elif tag == "link":
            if a.get("rel") == "canonical":
                self.canonical = a.get("href")
            elif "href" in a:
                self.links.append(a["href"])
        elif tag == "img":
            self.imgs.append(a)
            self.links.append(a.get("src", ""))
        elif tag == "meta":
            key = a.get("name") or a.get("property")
            if key:
                self.meta[key] = a.get("content", "")
        elif tag in ("h1", "h2", "h3", "h4", "h5", "h6"):
            self.headings.append(int(tag[1]))
        if tag in ("title", "script") and (tag == "title" or a.get("type") == "application/ld+json"):
            self._in, self._buf = tag, ""

    def handle_endtag(self, tag):
        if tag in VOID:
            return
        if not self.stack or self.stack[-1][0] != tag:
            opened = self.stack[-1] if self.stack else ("nothing", 0)
            problem("html", f"</{tag}> at line {self.getpos()[0]} closes <{opened[0]}> from line {opened[1]}")
            # Recover: pop to the matching tag if there is one.
            names = [t for t, _ in self.stack]
            if tag in names:
                while self.stack and self.stack.pop()[0] != tag:
                    pass
            return
        self.stack.pop()
        if tag == self._in:
            if tag == "title":
                self.title = self._buf.strip()
            else:
                self.jsonld.append(self._buf)
            self._in = None

    def handle_data(self, data):
        if self._in:
            self._buf += data


def own_repo_file(url, repo):
    """For a link to this repository's files on GitHub (blob/tree on main): the local path, else None."""
    for kind in ("/blob/main/", "/tree/main/"):
        if url.startswith(repo + kind):
            return os.path.join(REPO_ROOT, unquote(url[len(repo + kind):].split("#")[0]))
    return None


def check_site(out, external):
    with open(os.path.join(HERE, "site.json"), encoding="utf-8") as f:
        cfg = json.load(f)
    robots = open(os.path.join(out, "robots.txt"), encoding="utf-8").read()
    m = re.search(r"^Sitemap:\s*(\S+)", robots, re.M)
    if not m:
        problem("robots.txt", "no Sitemap line")
        return
    sitemap_url = m.group(1)
    base = sitemap_url[: -len("sitemap.xml")]
    root = urlparse(base).path
    if "Disallow: /\n" in robots:
        problem("robots.txt", "disallows everything")

    files = sorted(glob.glob(os.path.join(out, "**", "*.html"), recursive=True))
    titles, descs, indexable = {}, {}, []
    ext_links = set()
    for path in files:
        rel = os.path.relpath(path, out)
        url_path = "" if rel == "index.html" else rel[: -len("index.html")] if rel.endswith("/index.html") else rel
        url = base + url_path
        p = Page()
        global problems
        before = len(problems)
        p.feed(open(path, encoding="utf-8").read())
        for i in range(before, len(problems)):
            problems[i] = problems[i].replace("html:", f"{rel}:", 1)
        if p.stack:
            problem(rel, f"unclosed tags: {[t for t, _ in p.stack]}")
        is404 = rel == "404.html"
        if p.lang != "en":
            problem(rel, "html lang missing")
        if "viewport" not in p.meta:
            problem(rel, "no viewport meta")
        if not p.title:
            problem(rel, "no title")
        elif p.title in titles:
            problem(rel, f"title duplicates {titles[p.title]}")
        else:
            titles[p.title] = rel
        d = p.meta.get("description", "")
        if not (70 <= len(d) <= 170):
            problem(rel, f"meta description is {len(d)} characters (want 70-170)")
        if d in descs:
            problem(rel, f"description duplicates {descs[d]}")
        descs[d] = rel
        if not is404:
            if p.canonical != url:
                problem(rel, f"canonical {p.canonical} != {url}")
            indexable.append(url)
        for key in ("og:title", "og:description", "og:url", "og:image", "og:type", "twitter:card", "twitter:image"):
            if not p.meta.get(key):
                problem(rel, f"missing {key}")
        if p.meta.get("og:image", "").startswith("http") and not p.meta["og:image"].startswith(base):
            problem(rel, "og:image outside the site")
        if p.headings.count(1) != 1:
            problem(rel, f"{p.headings.count(1)} h1 elements (want 1)")
        last = 0
        for h in p.headings:
            if h > last + 1:
                problem(rel, f"heading h{h} after h{last} skips a level")
            last = h
        for img in p.imgs:
            if "alt" not in img:
                problem(rel, f"image {img.get('src')} has no alt")
        for block in p.jsonld:
            try:
                data = json.loads(block.replace("<\\/", "</"))
                if data.get("@context") != "https://schema.org":
                    problem(rel, "JSON-LD without schema.org context")
            except json.JSONDecodeError as e:
                problem(rel, f"JSON-LD doesn't parse: {e}")
        if not p.jsonld:
            problem(rel, "no JSON-LD")
        for href in p.links:
            if not href or href.startswith(("mailto:", "#")):
                continue
            u = urlparse(href)
            if u.scheme in ("http", "https"):
                if u.scheme == "http" and not u.netloc.startswith("localhost"):
                    problem(rel, f"insecure http link {href}")
                if href.startswith(base):
                    href = root + href[len(base):]
                else:
                    local_file = own_repo_file(href, cfg["repo"])
                    if local_file is not None:
                        # Checked against the working tree, so it also works before the docs are pushed.
                        if not os.path.exists(local_file):
                            problem(rel, f"link to a repository file that doesn't exist: {href}")
                    else:
                        ext_links.add(href.split("#")[0])
                    continue
            target = urljoin(root + url_path, href).split("#")[0]
            if not target.startswith(root):
                problem(rel, f"link outside the site root: {href}")
                continue
            local = os.path.join(out, unquote(target[len(root):]))
            if target.endswith("/"):
                local = os.path.join(local, "index.html")
            if not os.path.exists(local):
                problem(rel, f"broken internal link {href}")

    try:
        tree = ET.parse(os.path.join(out, "sitemap.xml"))
    except ET.ParseError as e:
        problem("sitemap.xml", f"doesn't parse: {e}")
        return
    ns = {"s": "http://www.sitemaps.org/schemas/sitemap/0.9"}
    locs = [e.text for e in tree.getroot().findall("s:url/s:loc", ns)]
    if sorted(locs) != sorted(indexable):
        problem("sitemap.xml", f"lists {sorted(set(locs) ^ set(indexable))} differently from the built pages")
    for e in tree.getroot().findall("s:url/s:lastmod", ns):
        if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", e.text or ""):
            problem("sitemap.xml", f"bad lastmod {e.text}")
    if cfg.get("cname") and not os.path.exists(os.path.join(out, "CNAME")):
        problem("CNAME", "site.json names a custom domain but no CNAME was written")
    print(f"site: {len(files)} pages, {len(locs)} in the sitemap, {len(ext_links)} external links")
    if external:
        check_external(ext_links, "site")


def github_slug(heading):
    s = heading.strip().lower()
    s = re.sub(r"[`*_]", "", s)
    s = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", s)
    s = re.sub(r"[^\w\- ]", "", s)
    return s.replace(" ", "-")


def anchors_of(md_path):
    text = open(md_path, encoding="utf-8").read()
    text = re.sub(r"```.*?```", "", text, flags=re.S)
    slugs, seen = set(), {}
    for h in re.findall(r"^#{1,6}\s+(.+?)\s*#*\s*$", text, re.M):
        s = github_slug(h)
        n = seen.get(s, 0)
        slugs.add(s if n == 0 else f"{s}-{n}")
        seen[s] = n + 1
    return slugs


def check_markdown(external):
    files = [os.path.join(REPO_ROOT, f) for f in ("README.md", "CONTRIBUTING.md", "SECURITY.md", "CODE_OF_CONDUCT.md")]
    files += sorted(glob.glob(os.path.join(REPO_ROOT, "docs", "*.md")))
    ext_links = set()
    count = 0
    for md in files:
        if not os.path.exists(md):
            continue
        rel_md = os.path.relpath(md, REPO_ROOT)
        text = open(md, encoding="utf-8").read()
        text_nocode = re.sub(r"```.*?```", "", text, flags=re.S)
        # Balanced parentheses inside a URL are allowed (CommonMark), e.g. File:PSVR2_(Non-Stereoscopic).png.
        links = re.findall(r"\]\(((?:[^()\s]|\([^()\s]*\))+)(?:\s+\"[^\"]*\")?\)", text_nocode)
        links += re.findall(r"<(https?://[^>\s]+)>", text_nocode)
        for link in links:
            count += 1
            if link.startswith(("http://", "https://")):
                ext_links.add(link.split("#")[0])
                continue
            if link.startswith("mailto:"):
                continue
            path, _, anchor = link.partition("#")
            target = md if not path else os.path.normpath(os.path.join(os.path.dirname(md), unquote(path)))
            if not os.path.exists(target):
                problem(rel_md, f"broken link {link}")
                continue
            if anchor and target.endswith(".md") and anchor not in anchors_of(target):
                problem(rel_md, f"anchor #{anchor} not found in {os.path.relpath(target, REPO_ROOT)}")
    print(f"markdown: {len(files)} files, {count} links, {len(ext_links)} external")
    if external:
        check_external(ext_links, "markdown")


def check_external(links, where):
    for url in sorted(links):
        req = urllib.request.Request(url, method="GET", headers={"User-Agent": "Mozilla/5.0 (link check)"})
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                if r.status >= 400:
                    problem(where, f"{url} returned {r.status}")
        except urllib.error.HTTPError as e:
            # Some sites refuse automated requests (403/429); report them separately, they're not proof of a dead link.
            if e.code in (401, 403, 429, 503):
                print(f"  (not checkable, {e.code}): {url}")
            else:
                problem(where, f"{url} returned {e.code}")
        except Exception as e:  # noqa: BLE001 - network errors of any kind
            print(f"  (unreachable: {e.__class__.__name__}): {url}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(HERE, "_site"))
    ap.add_argument("--markdown", action="store_true", help="also check the repository's Markdown links")
    ap.add_argument("--markdown-only", action="store_true", help="only check Markdown links")
    ap.add_argument("--external", action="store_true", help="also request external links (network)")
    args = ap.parse_args()
    if not args.markdown_only:
        check_site(args.out, args.external)
    if args.markdown or args.markdown_only:
        check_markdown(args.external)
    for p in problems:
        print("PROBLEM", p)
    print(f"{len(problems)} problems")
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
