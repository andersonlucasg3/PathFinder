using System;
using System.Collections.Generic;
using System.Threading;
using System.Text;

namespace PathFinding.AStar {
	public class PathFinder {
		private delegate void LoopHandler (Node neighbor);

		public static DebugMode DebugMode = DebugMode.DISABLED;

		private const int OrthogonalValue = 10;
		private const int DiagonalValue = 15;

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

		public void FindPathAsync(Node start, Node end) {
			if (runnerThread != null) {
				runnerThread.Abort ();
			}

			if (DebugMode != DebugMode.DISABLED) {
				benchmark = new DebugBenchmark ();
				benchmark.StartBenchmark ();
			}

			runnerThread = new Thread(new ThreadStart(delegate() {
				List<Node> path = StartPathFinding(start, end);

				DispatchFinish(path);
				runnerThread.Abort();
				runnerThread = null;
			}));
			runnerThread.Start ();
		}

		public List<Node> FindPathSync(Node start, Node end) {
			if (DebugMode != DebugMode.DISABLED) {
				benchmark = new DebugBenchmark();
				benchmark.StartBenchmark();
			}

			return StartPathFinding(start, end);
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

		private List<Node> StartPathFinding(Node start, Node end) {
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

				PrintDebugProcess (start, end, current, closedQueue);

				if (current == null) {
					break;
				}

				if (current.Equals(end)) {
					end.Parent = current;
					List<Node> path = ReconstructPath(end);

					PrintDebugResult (path.Count, start, end, current, closedQueue);

					return path;
				}
					
				closedQueue.Enqueue(openQueue.Dequeue());

				LoopNeighbors(current, (neighbor) => {
					ProcessCurrentNode(neighbor.Row, neighbor.Column, ref current, ref end,
											   ref closedQueue, ref openQueue);
				}); 
			}

			PrintDebugResult(0, start, end, null, closedQueue);

			return new List<Node>();
		}

		private void LoopNeighbors (Node current, LoopHandler step) {
			for (int row = current.Row - 1; row <= current.Row + 1; row++) {
				if (!(row < 0 || row == map.Length)) {
					for (int column = current.Column - 1; column <= current.Column + 1; column++) {
						if (!(column < 0 || column == map[row].Length)) {
							if (!ShouldSkipDiagonal(current.Row, current.Column, row, column)) {
								if (step != null) {
									step(map[row][column]);
								}
							}
						}
					}
				}
			}
		}

		private void ProcessCurrentNode(int row, int column, ref Node current, ref Node end, 
		                                ref Queue<Node> closedQueue, ref Queue<Node> openQueue) {
			Node neighbor = map[row][column];

			if (neighbor.Block == Block.WALL_BLOCK ||
				(!CanJump && neighbor.Block == Block.JUMPABLE_BLOCK) || 
			    closedQueue.Contains(neighbor)) {
				return;
			}

			int tentativeGScore = CalculateGScore(neighbor, current);

			if (!openQueue.Contains(neighbor)) {
				openQueue.Enqueue(neighbor);
			} else if (tentativeGScore >= neighbor.GScore) {
				return;
			}

			neighbor.Parent = current;
			neighbor.GScore = tentativeGScore;
			neighbor.FScore = neighbor.GScore + HeuristicEstimate(neighbor, end);
		}

		private bool ShouldSkipDiagonal(int currRow, int currColumn, int row, int column) {
			return !WalkDiagonals && (Math.Abs(currRow - row) + Math.Abs(currColumn - column) == 2);
		}

		private int CalculateGScore(Node neighbor, Node current) {
			int horizontal = Math.Abs(current.Column - neighbor.Column);
			int vertical = Math.Abs(current.Row - neighbor.Row);
			if (WalkDiagonals) {
				return current.GScore + ((horizontal != 0 && vertical != 0) ? DiagonalValue : OrthogonalValue);
			}

			return current.GScore + OrthogonalValue;
		}

		private void CalculateFScore(ref Node neighbor, Node current, Node end) {
			neighbor.FScore = neighbor.GScore + HeuristicEstimate(current, end);
		}

		private List<Node> ReconstructPath(Node endNode) {
			if (endNode.Parent != null) {
				List<Node> totalPath = new List<Node>();
				while (endNode.Parent != null) {
					totalPath.Add(endNode);
					endNode = endNode.Parent;
				}
				totalPath.Reverse();
				return totalPath;
			} 
			return null;
		}

		private int HeuristicEstimate(Node start, Node end) {
			switch (heuristic) {
			case Heuristic.Manhattan:
				return OrthogonalValue * (Math.Abs(start.Column - end.Column) + Math.Abs(start.Row - end.Row));

			case Heuristic.Diagonal:
				int xDist = Math.Abs(start.Column - end.Column);
				int yDist = Math.Abs(start.Row - end.Row);
				if (xDist > yDist) {
					return DiagonalValue * yDist + (xDist - yDist) * OrthogonalValue;
				} 
				return DiagonalValue * xDist + (yDist - xDist) * OrthogonalValue;

			case Heuristic.Euclidean:
				int dx = Math.Abs(start.Column - end.Column);
				int dy = Math.Abs(start.Row - end.Row);
				return (int)(OrthogonalValue * Math.Sqrt(dx * dx + dy * dy));
			}

			return 0;
		}

		private void PrintCurrentState(Node start, Node end, Node current, Queue<Node> closedQueue) {
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

					if (closedQueue.Contains(map[i][j])) {
						s[i][j] = 'C';
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

		private void PrintDebugProcess(Node start, Node end, Node current, Queue<Node> closedQueue) {
			PrintDebugProcess (start, end, current, closedQueue, false);
		}

		private void PrintDebugProcess(Node start, Node end, Node current, Queue<Node> closedQueue, bool force) {
			if (DebugMode == DebugMode.CONSOLE_LOG_PROGRESS || force) {
				PrintCurrentState(start, end, current, closedQueue);

				Thread.Sleep(100);
			}
		}

		private void PrintDebugResult(int count, Node start, Node end, Node current, Queue<Node> closedQueue) {
			if (DebugMode == DebugMode.CONSOLE_LOG_PROGRESS ||
				DebugMode == DebugMode.CONSOLE_LOG_RESULT) {
				benchmark.EndBenchmark ();

				PrintDebugProcess (start, end, current, closedQueue, true);

				Console.WriteLine();
				Console.WriteLine(string.Format("Path found {0}", count > 0));

				Console.WriteLine();
				Console.WriteLine(string.Format("Took {0} to complete", benchmark.Duration));
			}
		}
	}
}