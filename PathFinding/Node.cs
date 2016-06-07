using System;

namespace PathFinding {
	public class Node {
		internal int GScore {
			get;
			set;
		}

		internal int FScore {
			get;
			set;
		}

		internal int HScore {
			get { return FScore - GScore; }
		}

		internal Block Block {
			get;
			set;
		}

		internal Node Parent {
			get;
			set;
		}

		public int Column {
			get;
			private set;
		}

		public int Row {
			get;
			private set;
		}

		public Node(int row, int column) {
			Block = Block.EMPTY_BLOCK;
			Column = column;
			Row = row;
		}

		internal Node(Block block, int row, int column) {
			Block = block;
			Column = column;
			Row = row;
		}

		public override bool Equals(object obj) {
			if (obj is Node) {
				Node other = obj as Node;
				return Column == other.Column && Row == other.Row;
			}
			return base.Equals(obj);
		}
	}
}