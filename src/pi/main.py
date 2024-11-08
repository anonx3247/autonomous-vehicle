from pathfinding_logic.follow_line import follow_line
from pathfinding_logic.livraison import main
import sys

if __name__ == "__main__":
    if len(sys.argv) > 1:
        main() if sys.argv[1] != 'follow' else follow_line()
    else:
        main()


