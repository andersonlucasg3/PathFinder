using System;
using System.Collections.Generic;

namespace PathFinding {
	public class PathEventArgs : EventArgs {
		public List<Node> Path {
			get;
			private set;
		}

		internal PathEventArgs(List<Node> path) {
			Path = path;
		}
	}
}

