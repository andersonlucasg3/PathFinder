using System;
using PathFinding;
using PathFinding.AStar;

namespace PathFindingTest {
	class MainClass {
		private static PathFinder finder;

		public static void Main(string[] args) {
			Block[][] blocks = new Block[20][];
			for (int i = 0; i < blocks.Length; i++) {
				blocks[i] = new Block[20];
				for (int j = 0; j < blocks[i].Length; j++) {
					if ((i == 1 && j > 0 && j < blocks[i].Length - 1) ||
					    (i == blocks.Length - 2 && j > 0 && j < blocks[i].Length - 1) ||
					    (i == blocks.Length / 2 && j > 0 && j < blocks[i].Length - 1) ||
					    (j == blocks[i].Length / 2 && i > 0 && i < blocks[i].Length - 1) && 
					    i != blocks.Length / 4) {
						blocks[i][j] = Block.WALL_BLOCK;
					} else {
						blocks[i][j] = Block.EMPTY_BLOCK;
					}
				}
			}

			finder = new PathFinder(blocks);

			#if DEBUG
			PathFinder.DebugMode = DebugMode.CONSOLE_LOG_RESULT;
			#endif

			finder.CanJump = false;
			finder.WalkDiagonals = true;
			finder.Heuristic = Heuristic.Diagonal;

			StartFindPath(0, 0, blocks.Length * 3 / 4, blocks.Length * 3 / 4);

			finder.UpdateMap(Block.WALL_BLOCK, 5, 9);
			finder.UpdateMap(Block.WALL_BLOCK, 10, blocks.Length - 1);

			StartFindPath(0, 0, blocks.Length * 3 / 4, blocks.Length * 3 / 4);
			//StartFindPath(blocks.Length * 3 / 4, blocks.Length - 1, blocks.Length / 4, blocks.Length * 2 / 4);
		}

		private static void StartFindPath(int startX, int startY, int endX, int endY) {
			Node n1 = new Node(startX, startY);
			Node n2 = new Node(endX, endY);
			finder.FindPathSync(n1, n2);
		}
	}
}