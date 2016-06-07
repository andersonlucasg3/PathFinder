using System;

namespace PathFinding
{
	public class DebugBenchmark
	{
		private DateTime epoch = new DateTime(1970, 1, 1, 0, 0, 0, 0).ToLocalTime();

		private double startTime;
		private double endTime;

		public double Duration {
			get {
				return endTime - startTime;
			}
		}

		public DebugBenchmark() {
			startTime = 0;
			endTime = 0;
		}

		public void StartBenchmark() {
			startTime = GetCurrentTimeStamp();
		}

		public void EndBenchmark() {
			endTime = GetCurrentTimeStamp();
		}

		private double GetCurrentTimeStamp()
		{
			TimeSpan span = (DateTime.Now.ToLocalTime() - epoch);
			return span.TotalSeconds;
		}
	}
}

