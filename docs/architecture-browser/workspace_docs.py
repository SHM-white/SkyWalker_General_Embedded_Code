#!/usr/bin/env python3
"""Discover workspace Markdown for the local browser and static deployments."""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
from urllib.parse import urlsplit


INDEX_PATH = "/docs/architecture-browser/docs-index.json"
EXCLUDED_DIRS = {
    "node_modules", "build", "dist", "_site", "target", "vendor",
    "__pycache__", "venv", "env", "coverage", "htmlcov",
}
DOC_GROUPS = {
    "getting-started": "入门与配置", "modules": "模块手册",
    "applications": "应用说明", "guides": "工程指南",
    "dev": "开发记录", "api": "接口参考", "architecture-browser": "浏览器维护",
}


def title_of(path):
    """Prefer the first Markdown heading, with a filename fallback."""
    try:
        with path.open(encoding="utf-8-sig", errors="replace") as stream:
            lines = stream.read(65536).splitlines()
    except OSError:
        return path.stem
    fence = None
    previous = ""
    for line in lines:
        marker = re.match(r"^ {0,3}(`{3,}|~{3,})", line)
        if marker:
            token = marker[1]
            if fence is None:
                fence = token
            elif token[0] == fence[0] and len(token) >= len(fence):
                fence = None
            previous = ""
            continue
        if fence:
            continue
        heading = re.match(r"^ {0,3}#{1,6}\s+(.+?)\s*#*\s*$", line)
        value = heading[1] if heading else previous if previous and re.fullmatch(r" {0,3}(?:=+|-+)\s*", line) else None
        if value:
            value = re.sub(r"!?\[([^\]]+)\]\([^)]*\)", r"\1", value)
            return re.sub(r"[`*_]", "", value).strip() or path.stem
        previous = line.strip()
    return path.stem


def group_of(relative):
    parts = relative.parts
    if len(parts) == 1:
        return "仓库总览"
    if parts[0] == "docs":
        return DOC_GROUPS.get(parts[1], "文档 / " + parts[1]) if len(parts) > 2 else "文档入口与迁移说明"
    if parts[0] == "samples":
        return "样例 / " + parts[1] if len(parts) > 2 else "样例"
    if parts[0] == "tests":
        return "测试说明"
    return "工作区 / " + str(relative.parent).replace(os.sep, "/")


def scan(root, mode):
    documents = []

    def fail(error):
        raise error

    for directory, folders, files in os.walk(root, followlinks=False, onerror=fail):
        base = Path(directory)
        folders[:] = sorted(name for name in folders if not (
            name.startswith(".") or name in EXCLUDED_DIRS or name.startswith("build-")
            or (base / name).is_symlink()
        ))
        for name in sorted(files):
            path = base / name
            if name.startswith(".") or name == "AGENTS.md" or path.suffix.lower() != ".md" or path.is_symlink():
                continue
            relative = path.relative_to(root)
            route = relative.as_posix()
            if "\\" in route or any(ord(char) < 32 for char in route):
                continue
            document = {"title": title_of(path), "path": route, "group": group_of(relative)}
            if relative.parts[:2] == ("docs", "api"):
                with path.open(encoding="utf-8-sig", errors="replace") as stream:
                    document["searchText"] = stream.read(262144)
            documents.append(document)
    documents.sort(key=lambda document: (document["group"], document["path"]))
    return {"mode": mode, "documents": documents}


class WorkspaceHandler(SimpleHTTPRequestHandler):
    def serve_index(self, head=False):
        if urlsplit(self.path).path != INDEX_PATH:
            return False
        try:
            body = (json.dumps(scan(Path(self.directory), "workspace"), ensure_ascii=False) + "\n").encode("utf-8")
        except OSError as error:
            self.log_error("Markdown scan failed: %s", error)
            self.send_error(500, "Markdown scan failed")
            return True
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if not head:
            self.wfile.write(body)
        return True

    def do_GET(self):
        if not self.serve_index():
            super().do_GET()

    def do_HEAD(self):
        if not self.serve_index(head=True):
            super().do_HEAD()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--serve", type=int, metavar="PORT")
    action.add_argument("--output", type=Path, help="Write a static JSON index for the deployment tree")
    args = parser.parse_args()
    root = args.root.resolve()
    if not root.is_dir():
        parser.error("--root must be a workspace directory")
    if args.serve is not None:
        if not 1 <= args.serve <= 65535:
            parser.error("PORT must be between 1 and 65535")
        handler = partial(WorkspaceHandler, directory=str(root))
        with ThreadingHTTPServer(("127.0.0.1", args.serve), handler) as server:
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
    else:
        result = scan(root, "snapshot")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"Generated Markdown index: {len(result['documents'])} documents → {args.output}")


if __name__ == "__main__":
    main()
