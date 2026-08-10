from __future__ import annotations

from .cli import build_arg_parser, run_cli


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return run_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
