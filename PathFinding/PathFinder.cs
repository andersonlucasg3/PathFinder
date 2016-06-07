using System;
using System.Collections.Generic;
using System.Threading;
using System.Text;

namespace PathFinding.AStar {
	public class PathFinder {
		public static bool DebugMode = false;

		private const int OrthogonalValue = 10;
		private const int DiagonalValue = 14;

		private Node[][] map;
		private Heuristic heuristic;

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
			new Thread(new ThreadStart(delegate() {
				StartPathFinding(start, end);
			})).Start();
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

				if (DebugMode) {
					PrintCurrentState(start, end, current);

					Thread.Sleep(100);
				}

				if (current == null) {
					break;
				}

				if (current.Equals(end)) {
					DispatchFinish(ReconstructPath(current));
					return;
				}
					
				closedQueue.Enqueue(openQueue.Dequeue());

				for (int row = current.Row - 1; row <= current.Row + 1; row++) {
					if (!(row < 0 || row == map.Length)) {
						for (int column = current.Column - 1; column <= current.Column + 1; column++) {
							if (!(column < 0 || column == map[row].Length)) {
								Node neighbor = map[row][column];

								bool shouldComplete = true;
								if (neighbor.Block == Block.WALL_BLOCK ||
								    (!CanJump && neighbor.Block == Block.JUMPABLE_BLOCK) ||
								    closedQueue.Contains(neighbor)) {
									shouldComplete = false;
								}

								if (shouldComplete) {
									CalculateGScore(ref neighbor, current);
									CalculateFScore(ref neighbor, current, end);

									if (!openQueue.Contains(neighbor)) {
										openQueue.Enqueue(neighbor);
									} else if (neighbor.GScore >= current.GScore) {
										shouldComplete = false;
									}

									if (shouldComplete) {
										neighbor.Parent = current;
									}
								}
							}
						}
					}
				}
			}

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
						s[i][j] = 'B';
						break;
					}
				}
			}
		
			s[start.Row][start.Column] = 'S';

			s[end.Row][end.Column] = 'E';

			if (current != null) {
				Node c = current;
				while (c.Parent != null) {
					s[c.Row][c.Column] = '*';
					c = c.Parent;
				}
			}

			Console.Clear();
			foreach (char[] cs in s) {
				Console.WriteLine(cs);
			}
		}
	}
}