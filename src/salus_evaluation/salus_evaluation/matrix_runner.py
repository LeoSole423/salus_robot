"""CLI that aggregates individually captured navigation evaluation bundles."""

import argparse
import sys

from .matrix import expand_matrix, write_matrix_artifacts


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matrix", help="strict matrix YAML")
    parser.add_argument("output_dir", help="directory for matrix artifacts")
    parser.add_argument("trial_dirs", nargs="+", help="one bundle directory per expanded trial")
    args = parser.parse_args(argv)
    cells = expand_matrix(args.matrix)
    if len(cells) != len(args.trial_dirs):
        parser.error(
            f"matrix expands to {len(cells)} trials but got "
            f"{len(args.trial_dirs)} bundles"
        )
    write_matrix_artifacts(args.output_dir, args.matrix, cells, args.trial_dirs)
    return 0


if __name__ == "__main__":  # pragma: no cover - console entry point
    sys.exit(main())
