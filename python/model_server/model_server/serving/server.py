"""policy.v1 serving; implementation shared with the model-server executable."""

from ..server import main, run

if __name__ == "__main__":
    main()

__all__ = ["main", "run"]
