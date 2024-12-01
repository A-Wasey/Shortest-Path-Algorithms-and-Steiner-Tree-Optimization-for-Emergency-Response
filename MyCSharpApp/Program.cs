using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

class Program
{
    
    public struct Point
    {
        public int X;
        public int Y;

        public Point(int x, int y)
        {
            X = x;
            Y = y;
        }

        public override bool Equals(object obj)
        {
            if (obj is Point p)
            {
                return X == p.X && Y == p.Y;
            }
            return false;
        }

        public override int GetHashCode() => (X, Y).GetHashCode();

        public static bool operator ==(Point a, Point b) => a.Equals(b);
        public static bool operator !=(Point a, Point b) => !(a == b);

    }

    public struct Edge
    {
        public Point From;
        public Point To;
        public int Weight;

        public Edge(Point from, Point to, int weight)
        {
            From = from;
            To = to;
            Weight = weight;
        }
    }

    class Graph
    {
        private Dictionary<Point, List<Edge>> adjList;

        public Graph()
        {
            adjList = new Dictionary<Point, List<Edge>>();
        }

        public void AddEdge(Edge edge)
        {
            if (!adjList.ContainsKey(edge.From))
                adjList[edge.From] = new List<Edge>();
            if (!adjList.ContainsKey(edge.To))
                adjList[edge.To] = new List<Edge>();

            adjList[edge.From].Add(edge);
            adjList[edge.To].Add(edge);
        }

        public List<Edge> Dijkstra(Point start, Point target)
        {
            var dist = new Dictionary<Point, int>();
            var prev = new Dictionary<Point, Point>();
            var pq = new SortedSet<Node>(Comparer<Node>.Create((a, b) => a.Dist == b.Dist ? a.Point.GetHashCode().CompareTo(b.Point.GetHashCode()) : a.Dist.CompareTo(b.Dist)));

            foreach (var vertex in adjList.Keys)
            {
                dist[vertex] = int.MaxValue;
                prev[vertex] = new Point(-1, -1);
            }

            dist[start] = 0;
            pq.Add(new Node(0, start));

            while (pq.Any())
            {
                var current = pq.Min;
                pq.Remove(current);

                if (current.Point.Equals(target))
                    break;

                if (dist[current.Point] < current.Dist)
                    continue;

                foreach (var edge in adjList[current.Point])
                {
                    var neighbor = edge.From.Equals(current.Point) ? edge.To : edge.From;
                    int alt = dist[current.Point] + edge.Weight;

                    if (alt < dist[neighbor])
                    {
                        dist[neighbor] = alt;
                        prev[neighbor] = current.Point;
                        pq.Add(new Node(dist[neighbor], neighbor));
                    }
                }
            }

            List<Edge> path = new List<Edge>();
            Point pathNode = target;

            while (!pathNode.Equals(start))
            {
                var prevNode = prev[pathNode];
                var edge = adjList[pathNode].First(e => e.From.Equals(pathNode) && e.To.Equals(prevNode) || e.To.Equals(pathNode) && e.From.Equals(prevNode));
                path.Insert(0, edge);
                pathNode = prevNode;
            }

            return path;
        }

        public Dictionary<Point, Dictionary<Point, int>> GetAllPairsShortestPaths(List<Point> terminals)
        {
            var allShortestPaths = new Dictionary<Point, Dictionary<Point, int>>();

            foreach (var start in terminals)
            {
                allShortestPaths[start] = new Dictionary<Point, int>();
                foreach (var target in terminals)
                {
                    if (!start.Equals(target))
                    {
                        var path = Dijkstra(start, target);
                        var totalWeight = path.Sum(edge => edge.Weight);
                        allShortestPaths[start][target] = totalWeight;
                    }
                }
            }

            return allShortestPaths;
        }

        public List<Edge> FindSteinerTree(List<Point> terminals, Dictionary<Point, Dictionary<Point, int>> allShortestPaths)
        {
            // Step 1: Create a graph for the Steiner Tree
            var steinerEdges = new List<Edge>();

            // Build the Steiner Graph using shortest paths
            foreach (var source in terminals)
            {
                foreach (var target in terminals)
                {
                    if (!source.Equals(target))
                    {
                        int weight = allShortestPaths[source][target];
                        steinerEdges.Add(new Edge(source, target, weight));
                    }
                }
            }

            // Step 2: Compute MST on the Steiner Graph using Kruskal's Algorithm
            steinerEdges = steinerEdges.OrderBy(e => e.Weight).ToList();
            var parent = new Dictionary<Point, Point>();
            var rank = new Dictionary<Point, int>();

            // Initialize union-find structures
            foreach (var terminal in terminals)
            {
                parent[terminal] = terminal;
                rank[terminal] = 0;
            }

            // Helper functions for Union-Find
            Point Find(Point p)
            {
                if (!parent[p].Equals(p))
                    parent[p] = Find(parent[p]);
                return parent[p];
            }

            void Union(Point u, Point v)
            {
                var rootU = Find(u);
                var rootV = Find(v);

                if (rootU.Equals(rootV)) return;

                if (rank[rootU] > rank[rootV])
                    parent[rootV] = rootU;
                else if (rank[rootU] < rank[rootV])
                    parent[rootU] = rootV;
                else
                {
                    parent[rootV] = rootU;
                    rank[rootU]++;
                }
            }

            // Step 3: Add edges to form the MST
            var mstEdges = new List<Edge>();

            foreach (var edge in steinerEdges)
            {
                if (Find(edge.From) != Find(edge.To))
                {
                    mstEdges.Add(edge);
                    Union(edge.From, edge.To);
                }
            }

            // Return the MST edges as the Steiner Tree
            return mstEdges;
        }


    }

    public struct Node
    {
        public int Dist { get; set; }
        public Point Point { get; set; }

        public Node(int dist, Point point)
        {
            Dist = dist;
            Point = point;
        }

        public override bool Equals(object obj)
        {
            if (obj is Node otherNode)
            {
                return Dist == otherNode.Dist && Point.Equals(otherNode.Point);
            }
            return false;
        }

        public override int GetHashCode()
        {
            return Dist.GetHashCode() ^ Point.GetHashCode();
        }
    }

    static void Main(string[] args)
    {

        string inputFolder = Path.Combine(Directory.GetCurrentDirectory(), "input_files");
        string[] files = Directory.GetFiles(inputFolder, "*.txt");
        Console.WriteLine("Available input files:");
        for (int i = 0; i < files.Length; i++)
        {
            Console.WriteLine($"{i + 1}. {Path.GetFileName(files[i])}");
        }

        Console.Write("Select a file by number: ");
        long initialMemory = GC.GetTotalMemory(false); //to measure memory metrics
        int selectedFileIndex = int.Parse(Console.ReadLine()) - 1;

        var stopwatch = System.Diagnostics.Stopwatch.StartNew();

        string filePath = files[selectedFileIndex];
        Graph graph = new Graph();
        Point start = new Point();
        List<Point> targets = new List<Point>();

        foreach (var line in File.ReadLines(filePath))
        {
            if (line.StartsWith("START"))
            {
                var parts = line.Split(' ')[1].Split(',');
                start = new Point(int.Parse(parts[0]), int.Parse(parts[1]));
            }
            else if (line.StartsWith("TARGETS"))
            {
                var parts = line.Split(' ');
                int targetCount = int.Parse(parts[1]);
                for (int i = 2; i < 2 + targetCount; i++)
                {
                    var coords = parts[i].Split(',');
                    targets.Add(new Point(int.Parse(coords[0]), int.Parse(coords[1])));
                }
            }
            else
            {
                var parts = line.Split(' ');
                if (parts.Length == 3)
                {
                    var fromCoords = parts[0].Split(',');
                    var toCoords = parts[1].Split(',');
                    int weight = int.Parse(parts[2]);

                    Point from = new Point(int.Parse(fromCoords[0]), int.Parse(fromCoords[1]));
                    Point to = new Point(int.Parse(toCoords[0]), int.Parse(toCoords[1]));

                    graph.AddEdge(new Edge(from, to, weight));
                }
            }
        }

        Console.WriteLine($"\nStarting Point: ({start.X}, {start.Y})");
        Console.WriteLine("Target Nodes: [" + string.Join(", ", targets.Select(t => $"({t.X}, {t.Y})")) + "]");

        Console.WriteLine("\nShortest Paths using Dijkstra's Algorithm:");
        foreach (var target in targets)
        {
            var path = graph.Dijkstra(start, target);
            int accumulatedWeight = path.Sum(e => e.Weight);
            Console.WriteLine($"\nPath from ({start.X}, {start.Y}) to ({target.X}, {target.Y}):");
            foreach (var edge in path)
            {
                Console.WriteLine($"Edge: ({edge.From.X}, {edge.From.Y}) -> ({edge.To.X}, {edge.To.Y}) (Weight: {edge.Weight})");
            }
            Console.WriteLine($"Total Weight: {accumulatedWeight}");
        }

        Console.WriteLine("\nEdges in the Steiner Tree:");
        var allShortestPaths = graph.GetAllPairsShortestPaths(new List<Point> { start }.Concat(targets).ToList());
        var steinerTree = graph.FindSteinerTree(new List<Point> { start }.Concat(targets).ToList(), allShortestPaths);

        int steinerAccumulatedWeight = steinerTree.Sum(e => e.Weight);
        foreach (var edge in steinerTree)
        {
            Console.WriteLine($"Edge: ({edge.From.X}, {edge.From.Y}) -> ({edge.To.X}, {edge.To.Y}) (Weight: {edge.Weight})");
        }
        Console.WriteLine($"Total Steiner Tree Weight: {steinerAccumulatedWeight}");

        long finalMemory = GC.GetTotalMemory(false);
        stopwatch.Stop();
        Console.WriteLine("\n");
        Console.WriteLine($"Execution Time: {stopwatch.ElapsedMilliseconds} ms\n");
        Console.WriteLine($"Memory Usage: {finalMemory - initialMemory} bytes\n");
    }
}
