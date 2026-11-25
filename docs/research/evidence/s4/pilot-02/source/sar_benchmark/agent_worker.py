"""Honest local policy worker. Process separation is not an OS security sandbox."""
import argparse
import sys

from .contracts import canonical, strict_loads, validate_observation
from .policies import SearchPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=("random", "sweep", "core"), required=True)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    if args.method == "core":
        from .core_policy import CoreSearchPolicy
        policy = CoreSearchPolicy(args.seed)
    else:
        policy = SearchPolicy(args.method, args.seed)
    try:
        for line in sys.stdin.buffer:
            obs = validate_observation(strict_loads(line))
            sys.stdout.buffer.write(canonical(policy.act(obs)) + b"\n")
            sys.stdout.buffer.flush()
    finally:
        if hasattr(policy, "close"):
            policy.close()


if __name__ == "__main__":
    main()
