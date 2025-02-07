using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Assets.TrackGeneration
{
    public class Graph
    {
        public Dictionary<Vector2, List<Vector2>> AdjacencyList { get; set; }
        public Dictionary<Vector2, bool> Visited { get; set; }
        public List<Cycle> cycles = new List<Cycle>();

        public Graph()
        {
            AdjacencyList = new Dictionary<Vector2, List<Vector2>>();
            Visited = new Dictionary<Vector2, bool>();
        }

        public void AddEdge(Vector2 start, Vector2 end)
        {
            if (!AdjacencyList.ContainsKey(start))
                AdjacencyList[start] = new List<Vector2>();
            if (!AdjacencyList.ContainsKey(end))
                AdjacencyList[end] = new List<Vector2>();

            AdjacencyList[start].Add(end);
            AdjacencyList[end].Add(start);

            if (!Visited.ContainsKey(start))
                Visited[start] = false;
            if (!Visited.ContainsKey(end))
                Visited[end] = false;
        }

        public bool HasEdge(Vector2 start, Vector2 end)
        {
            if (AdjacencyList.ContainsKey(start) && AdjacencyList.ContainsKey(end))
            {
                return AdjacencyList[start].Any(v => Vector2.Distance(v, end) < 0.001f) ||
                       AdjacencyList[end].Any(v => Vector2.Distance(v, start) < 0.001f);
            }
            return false;
        }

        public void MarkVisited(Vector2 point)
        {
            if (Visited.ContainsKey(point))
            {
                Visited[point] = true;
            }
        }

        public bool IsVisited(Vector2 point)
        {
            return Visited.ContainsKey(point) && Visited[point];
        }

        public List<Cycle> FindAllCycles()
        {
            cycles.Clear();
            Dictionary<Vector2, Vector2> parent = new Dictionary<Vector2, Vector2>();
            HashSet<Vector2> currentPath = new HashSet<Vector2>();

            // Reset visited states
            Vector2[] vertices = new Vector2[Visited.Count];
            Visited.Keys.CopyTo(vertices, 0);
            foreach (var vertex in vertices)
            {
                Visited[vertex] = false;
            }

            // Start DFS from each unvisited vertex
            foreach (var vertex in AdjacencyList.Keys)
            {
                if (!IsVisited(vertex))
                {
                    currentPath.Clear();
                    DFSForCycles(vertex, vertex, parent, currentPath);
                }
            }

            System.Random random = new System.Random();
            int rnd = random.Next(cycles.Count);
            List<Cycle> rcycles = new List<Cycle> { cycles[rnd] };
            return cycles;
        }

        private void DFSForCycles(Vector2 current, Vector2 caller, Dictionary<Vector2, Vector2> parent, HashSet<Vector2> currentPath)
        {
            MarkVisited(current);
            currentPath.Add(current);
            parent[current] = caller;

            foreach (Vector2 neighbor in AdjacencyList[current])
            {
                if (Vector2.Distance(neighbor, caller) < 0.001f)
                    continue;

                if (!IsVisited(neighbor))
                {
                    DFSForCycles(neighbor, current, parent, currentPath);
                }
                else if (currentPath.Contains(neighbor))
                {
                    List<Vector2> cyclePoints = new List<Vector2>();
                    Vector2 temp = current;

                    while (Vector2.Distance(temp, neighbor) > 0.001f)
                    {
                        cyclePoints.Add(temp);
                        temp = parent[temp];
                    }
                    cyclePoints.Add(neighbor);
                    cyclePoints.Add(current);

                    if (!IsDuplicateCycle(cyclePoints))
                    {
                        Cycle cycle = new Cycle(cyclePoints);
                        cycles.Add(cycle);
                    }
                }
            }

            currentPath.Remove(current);
        }

        private bool IsDuplicateCycle(List<Vector2> newCycle)
        {
            foreach (Cycle existingCycle in cycles)
            {
                if (existingCycle.Points.Count != newCycle.Count)
                    continue;

                // Try matching the cycle from each possible starting point
                for (int i = 0; i < existingCycle.Points.Count; i++)
                {
                    bool isMatch = true;
                    for (int j = 0; j < newCycle.Count; j++)
                    {
                        int index = (i + j) % existingCycle.Points.Count;
                        if (Vector2.Distance(existingCycle.Points[index], newCycle[j]) > 0.001f)
                        {
                            isMatch = false;
                            break;
                        }
                    }
                    if (isMatch)
                        return true;

                    // Also check reverse direction
                    isMatch = true;
                    for (int j = 0; j < newCycle.Count; j++)
                    {
                        int index = (i - j + existingCycle.Points.Count) % existingCycle.Points.Count;
                        if (Vector2.Distance(existingCycle.Points[index], newCycle[j]) > 0.001f)
                        {
                            isMatch = false;
                            break;
                        }
                    }
                    if (isMatch)
                        return true;
                }
            }
            return false;
        }

        public List<Cycle> SortCycles(int minLength, int maxLength, int minAngle)
        {
            List<Cycle> longestCycle = cycles
                .Where(c => c.MinAngle >= minAngle)
                .Where(c => c.Length <= maxLength && c.Length >= minLength)
                .Where(c => c.Overlaps == false)
                .Where(c => c.Corners > 4)
                .OrderByDescending(c => c.Length)
                .ToList();
            List<Cycle> firstCycle = new List<Cycle> { longestCycle[0] };
            return firstCycle;
        }

        public void RemoveOutOfBounds(int canvas)
        {
            List<Vector2> pointsToRemove = new List<Vector2>();

            foreach (var point in AdjacencyList.Keys)
            {
                if (point.x < 0 || point.x > canvas || point.y < 0 || point.y > canvas)
                {
                    pointsToRemove.Add(point);
                }
            }

            foreach (var point in pointsToRemove)
            {
                if (AdjacencyList.ContainsKey(point))
                {
                    foreach (var neighbor in AdjacencyList[point])
                    {
                        AdjacencyList[neighbor].Remove(point);
                    }
                    AdjacencyList.Remove(point);
                }
            }
        }
    }
}