using System;
using PathFinding;
using PathFinding.AStar;

namespace PathFindingTest {
	class MainClass {
		public static void Main(string[] args) {
			Block[][] blocks = new Block[20][];
			for (int i = 0; i < blocks.Length; i++) {
				blocks[i] = new Block[20];
				for (int j = 0; j < blocks[i].Length; j++) {
					if ((i == 1 && j > 0 && j < blocks[i].Length - 1) ||
					    (i == blocks.Length - 2 && j > 0 && j < blocks[i].Length - 1)) {
						blocks[i][j] = Block.WALL_BLOCK;
					} else {
						blocks[i][j] = Block.EMPTY_BLOCK;
					}
				}
			}

			bool finishProgram = false;

			PathFinder finder = new PathFinder(blocks);
			#if DEBUG
			PathFinder.DebugMode = true;
			#endif

			finder.CanJump = false;
			finder.WalkDiagonals = true;

			double startTime = CreatedEpoch();

			finder.FindPathFinished += (object sender, PathEventArgs e) => {
				Console.WriteLine();
				Console.WriteLine();
				Console.WriteLine(String.Format("Path found {0}", e.Path.Count > 0));

				Console.WriteLine();
				Console.WriteLine();

				Console.WriteLine(String.Format("Took {0} to complete", CreatedEpoch() - startTime));

				finishProgram = true;
			};

			finder.FindPath(new Node(0, 0), new Node(blocks.Length - 1, blocks[0].Length - 1));

			while (!finishProgram)
				;
		}

		public static double CreatedEpoch()
		{
			DateTime epoch = new DateTime(1970, 1, 1, 0, 0, 0, 0).ToLocalTime();
			TimeSpan span = (DateTime.Now.ToLocalTime() - epoch);
			return span.TotalSeconds;
		}
	}
}