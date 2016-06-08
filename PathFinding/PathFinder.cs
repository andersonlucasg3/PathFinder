using System;
using System.Collections.Generic;
using System.Threading;
using System.Text;

namespace PathFinding.AStar {
	public class PathFinder {
		public static DebugMode DebugMode = DebugMode.DISABLED;

		private const int OrthogonalValue = 10;
		private const int DiagonalValue = 14;

		private Node[][] map;
		private Heuristic heuristic;

		private Thread runnerThread;
		private DebugBenchmark benchmark;

		public Heuristic Heuristic {
			get { return heuristic; }
			set { heuristic = value; }
		}

		public bool WalkDiagonals {
			get;
			set;
		}

		public bool CanJump {
			get;
			set;
		}

		public event EventHandler<PathEventArgs> FindPathFinished;

		public PathFinder(Block[][] blocks) {
			heuristic = Heuristic.Manhattan;
			GenerateNodeMap(blocks);
		}

		public void UpdateMap(Block block, int row, int column) {
			if (row > -1 && column > -1 &&
			    row < map.Length && column < map[column].Length) {
				
				UpdateNodeMap(block, row, column);
			}
		}

		public void FindPath(Node start, Node end) {
			if (runnerThread != null) {
				runnerThread.Abort ();
			}

			if (DebugMode != DebugMode.DISABLED) {
				benchmark = new DebugBenchmark ();
				benchmark.StartBenchmark ();
			}

			runnerThread = new Thread(new ThreadStart(delegate() {
				StartPathFinding(start, end);
			}));
			runnerThread.Start ();
		}

		private void GenerateNodeMap(Block[][] blocks) {
			int rows = blocks.Length;
			int columns = blocks[0].Length;
			map = new Node[rows][];

			for (int row = 0; row < rows; row++) {
				map[row] = new Node[columns];

				for (int column = 0; column < columns; column++) {
					map[row][column] = new Node(blocks[row][column], row, column);
				}
			}
		}

		private void UpdateNodeMap(Block block, int row, int column) {
			map[row][column] = new Node(block, row, column);
		}

		private void DispatchFinish(List<Node> path) {
			EventHandler<PathEventArgs> handler = FindPathFinished;
			if (handler != null) {
				handler(this, new PathEventArgs(path));
			}
		}

		private void StartPathFinding(Node start, Node end) {
			if (start.Block == Block.WALL_BLOCK || end.Block == Block.WALL_BLOCK) {
				DispatchFinish(new List<Node>());
			}

			Queue<Node> closedQueue = new Queue<Node>();

			Queue<Node> openQueue = new Queue<Node>();
			openQueue.Enqueue(start);

			start.FScore = HeuristicEstimate(start, end);
			start.GScore = 0;

			while (openQueue.Count > 0) {
				Node current = openQueue.Peek();

				PrintDebugProcess (start, end, current);

				if (current == null) {
					break;
				}

				if (current.Equals(end)) {
					List<Node> path = ReconstructPath (current);

					PrintDebugResult (path.Count, start, end, current);

					DispatchFinish(path);
					runnerThread.Abort ();
					runnerThread = null;
					return;
				}
					
				closedQueue.Enqueue(openQueue.Dequeue());

			}

			PrintDebugResult(0, start, end, null);

			DispatchFinish(new List<Node>());
		}

		private void CalculateGScore(ref Node neighbor, Node current) {
			int horizontal = System.Math.Abs(current.Column - neighbor.Column);
			int vertical = System.Math.Abs(current.Row - neighbor.Row);
			if (WalkDiagonals) {
				neighbor.GScore = current.GScore + ((horizontal != 0 && vertical != 0) ? DiagonalValue : OrthogonalValue);
			} else {
				neighbor.GScore = current.GScore + OrthogonalValue;
			}
		}

		private void CalculateFScore(ref Node neighbor, Node current, Node end) {
			neighbor.FScore = neighbor.GScore + HeuristicEstimate(current, end);
		}

		private List<Node> ReconstructPath(Node endNode) {
			List<Node> totalPath = new List<Node>();
			while (endNode.Parent != null) {
				totalPath.Add(endNode);
				endNode = endNode.Parent;
			}
			totalPath.Reverse();
			return totalPath;
		}

		private int HeuristicEstimate(Node start, Node end) {
			return (System.Math.Abs(start.Column - end.Column) * OrthogonalValue + System.Math.Abs(start.Row - end.Row) * OrthogonalValue);
		}

		private void PrintCurrentState(Node start, Node end, Node current) {
			int rows = map.Length;
			int columns = map[0].Length;

			char[][] s = new char[rows][];

			for (int i = 0; i < rows; i++) {
				s[i] = new char[columns];
				for (int j = 0; j < columns; j++) {
					switch (map[i][j].Block) {
					case Block.EMPTY_BLOCK:
						s[i][j] = '0';
						break;

					case Block.JUMPABLE_BLOCK:
						s[i][j] = 'J';
						break;

					case Block.WALL_BLOCK:
						s[i][j] = '|';
						break;
					}
				}
			}
		
			s[start.Row][start.Column] = 'S';

			s[end.Row][end.Column] = 'E';

			if (current != null) {
				Node c = current;
				while (c.Parent != null) {
					if (!c.Equals(start) && !c.Equals(end)) {
						s[c.Row][c.Column] = '*';
					}
					c = c.Parent;
				}
			}

			Console.Clear();
			foreach (char[] cs in s) {
				Console.WriteLine(cs);
			}
		}

		private void PrintDebugProcess(Node start, Node end, Node current) {
			PrintDebugProcess (start, end, current, false);
		}

		private void PrintDebugProcess(Node start, Node end, Node current, bool force) {
			if (DebugMode == DebugMode.CONSOLE_LOG_PROGRESS || force) {
				PrintCurrentState(start, end, current);

				Thread.Sleep(100);
			}
		}

		private void PrintDebugResult(int count, Node start, Node end, Node current) {
			if (DebugMode == DebugMode.CONSOLE_LOG_PROGRESS ||
				DebugMode == DebugMode.CONSOLE_LOG_RESULT) {
				benchmark.EndBenchmark ();

				PrintDebugProcess (start, end, current, true);

				Console.WriteLine();
				Console.WriteLine();
				Console.WriteLine(String.Format("Path found {0}", count > 0));

				Console.WriteLine();
				Console.WriteLine();

				Console.WriteLine(String.Format("Took {0} to complete", benchmark.Duration));
			}
		}
	}
}