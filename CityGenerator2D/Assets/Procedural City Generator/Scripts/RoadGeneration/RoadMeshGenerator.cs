using System;
using System.Collections.Generic;
using System.Linq;
using GraphModel;
using Services;
using UnityEngine;
using BlockGeneration;

namespace RoadGeneration
{
    class RoadMeshGenerator
    {
        public class HalfEdgedNode
        {
            public float X { get; private set; }
            public float Y { get; private set; }
            public List<HalfEdgedEdge> Edges { get; private set; } //이 정점에서 시작하는 간선들
            public HalfEdgedNode(float x, float y)
            {
                X = x;
                Y = y;

                Edges = new List<HalfEdgedEdge>();
            }
            public void AddEdge(HalfEdgedEdge edge)
            {
                Edges.Add(edge);
            }

            public HalfEdgedEdge MostRightEdge(HalfEdgedEdge path)
            {
                if (path.NodeEnd != this) throw new NotSupportedException("이 정점에 연결되어 있는 간선이 아닙니다.");
                HalfEdgedEdge mostRightEdge = null;
                float angle = 0f;
                foreach (HalfEdgedEdge edge in Edges)
                {
                    if (edge.NodeEnd == path.NodeStart) continue;
                    if (mostRightEdge == null)
                    {
                        mostRightEdge = edge;
                        angle = (float)((edge.DirRadianFromStart - path.DirRadianFromStart + 3 * Math.PI) % (2 * Math.PI));
                    }
                    else if(((edge.DirRadianFromStart - path.DirRadianFromStart + 3 * Math.PI) % (2 * Math.PI)) < angle)
                    {
                        mostRightEdge = edge;
                        angle = (float)((edge.DirRadianFromStart - path.DirRadianFromStart + 3 * Math.PI) % (2 * Math.PI));
                    }
                }
                return mostRightEdge;
            }
        }
        public class HalfEdgedEdge
        {
            public HalfEdgedNode NodeStart { get; private set; }
            public HalfEdgedNode NodeEnd { get; private set; }
            public float DirRadianFromStart { get; private set; }
            public float DirRadianFromEnd { get; private set; }

            protected HalfEdgedEdge(HalfEdgedNode first, HalfEdgedNode second)
            {
                NodeStart = first;
                NodeEnd = second;

                DirRadianFromStart = Mathf.Atan2(second.Y - first.Y, second.X - first.X);
                DirRadianFromEnd = Mathf.Atan2(first.Y - second.Y, first.X - second.X);
            }
            public static void AddEdge(HalfEdgedNode node1, HalfEdgedNode node2)
            {
                if (node1 == node2) throw new NotSupportedException("두 정점이 같으면 연결할 수 없습니다.");
                HalfEdgedEdge edge1 = new HalfEdgedEdge(node1, node2);
                HalfEdgedEdge edge2 = new HalfEdgedEdge(node2, node1);
                node1.AddEdge(edge1);
                node2.AddEdge(edge2);
            }
        }
        
        public class Triangle
        {
            public HalfEdgedNode p1 { get; protected set; }
            public HalfEdgedNode p2 { get; protected set; }
            public HalfEdgedNode p3 { get; protected set; }

            public Triangle(HalfEdgedNode a, HalfEdgedNode b, HalfEdgedNode c)
            {
                this.p1 = a;
                this.p2 = b;
                this.p3 = c;
            }

            internal bool isClockwise()
            {
                bool isClockWise = true;

                float determinant = p1.X * p2.Y + p3.X * p1.Y + p2.X * p3.Y - p1.X * p3.Y - p3.X * p2.Y - p2.X * p1.Y;

                if (determinant > 0f)
                {
                    isClockWise = false;
                }

                return isClockWise;
            }
        }
        public List<HalfEdgedNode> Nodes { get; private set; }

        public List<List<HalfEdgedNode>> polygons { get; private set; }

        private readonly Graph graph;
        public List<Block> Blocks { get; private set; }
        private readonly int border;


        public RoadMeshGenerator(Graph graphToUse, int mapSize)
        {
            graph = graphToUse;
            Blocks = new List<Block>();
            Nodes = new List<HalfEdgedNode>();
            polygons = new List<List<HalfEdgedNode>>();
            border = mapSize;
            foreach(Node node in graph.MajorNodes)
            {
                Nodes.Add(new HalfEdgedNode(node.X, node.Y));
            }
            foreach (Node node in graph.MinorNodes)
            {
                Nodes.Add(new HalfEdgedNode(node.X, node.Y));
            }
            foreach (Edge edge in graph.MajorEdges)
            {
                HalfEdgedNode nodeA = Nodes.Find(x => x.X == edge.NodeA.X && x.Y == edge.NodeA.Y);
                HalfEdgedNode nodeB = Nodes.Find(x => x.X == edge.NodeB.X && x.Y == edge.NodeB.Y);
                HalfEdgedEdge.AddEdge(nodeA, nodeB);
            }
            foreach (Edge edge in graph.MinorEdges)
            {
                HalfEdgedNode nodeA = Nodes.Find(x => x.X == edge.NodeA.X && x.Y == edge.NodeA.Y);
                HalfEdgedNode nodeB = Nodes.Find(x => x.X == edge.NodeB.X && x.Y == edge.NodeB.Y);
                HalfEdgedEdge.AddEdge(nodeA, nodeB);
            }
            foreach(HalfEdgedNode node in Nodes)
            {
                while (FindHole(node))
                {
                    //도형을 찾는다.(여기는 빈 블록)
                }
            }
            Debug.Log("Total terrain polygon number" + polygons.Count);
        }

        public Mesh GenerateMesh()
        {
            Mesh IMesh = new Mesh();
            List<Vector3> vertices = new List<Vector3>();  //Not the most optimized way, because same vertex can be stored more than once
            List<int> triangles = new List<int>();
            foreach(List<HalfEdgedNode> polygon in polygons)
            {
                List<Triangle> tris = TriangulatePolygon(polygon);
                foreach(Triangle tri in tris)
                {
                    Vector3 a = new Vector3(tri.p1.X, Mathf.PerlinNoise(tri.p1.X, tri.p1.Y) * 1, tri.p1.Y);
                    Vector3 b = new Vector3(tri.p2.X, Mathf.PerlinNoise(tri.p2.X, tri.p2.Y) * 1, tri.p2.Y);
                    Vector3 c = new Vector3(tri.p3.X, Mathf.PerlinNoise(tri.p3.X, tri.p3.Y) * 1, tri.p3.Y);

                    vertices.Add(a);
                    vertices.Add(b);
                    vertices.Add(c);

                    triangles.Add(vertices.IndexOf(a));
                    triangles.Add(vertices.IndexOf(b));
                    triangles.Add(vertices.IndexOf(c));
                }
            }
            IMesh.vertices = vertices.ToArray();
            IMesh.triangles = triangles.ToArray();
            return IMesh;
        }

        public static List<Triangle> TriangulatePolygon(List<HalfEdgedNode> points)
        {
            //The list with triangles the method returns
            List<Triangle> tris = new List<Triangle>();

            //If we just have three points, then we dont have to do all calculations
            if (points.Count == 3)
            {
                tris.Add(new Triangle(points[0], points[1], points[2]));

                return tris;
            }

            while (true)
            {
                //This means we have just one triangle left
                if (points.Count == 3)
                {
                    //The final triangle
                    tris.Add(new Triangle(points[0], points[1], points[2]));
                    break;
                }
                if (IsVertexEar(points, points[points.Count - 1], points[0], points[1]))
                {
                    tris.Add(new Triangle(points[points.Count - 1], points[0], points[1]));
                    points.RemoveAt(0);
                    continue;
                }
                if (IsVertexEar(points, points[points.Count - 2], points[points.Count - 1], points[0]))
                {
                    tris.Add(new Triangle(points[points.Count - 2], points[points.Count - 1], points[0]));
                    points.RemoveAt(points.Count - 1);
                    continue;
                }
                for (int i = 1; i < points.Count - 1; i++)
                {
                    if (IsVertexEar(points, points[i], points[i - 1], points[i + 1]))
                    {
                        tris.Add(new Triangle(points[i - 1], points[i], points[i + 1]));
                        points.RemoveAt(i);
                        break;
                    }
                }
            }

            //Debug.Log(triangles.Count);

            return tris;
        }


        //Check if a vertex is an ear
        protected static bool IsVertexEar(List<HalfEdgedNode> polygon, HalfEdgedNode prev_v, HalfEdgedNode v, HalfEdgedNode next_v)
        {
            //A reflex vertex cant be an ear!
            if (!new Triangle(prev_v, v, next_v).isClockwise())
            {
                return false;
            }

            //This triangle to check point in triangle
            Vector2 a = new Vector2(prev_v.X, prev_v.Y);
            Vector2 b = new Vector2(v.X, v.Y);
            Vector2 c = new Vector2(next_v.X, next_v.Y);

            bool hasPointInside = false;

            for (int i = 0; i < polygon.Count; i++)
            {
                Vector2 p = new Vector2(polygon[i].X, polygon[i].Y);

                //This means inside and not on the hull
                if (IsPointInTriangle(a, b, c, p))
                {
                    hasPointInside = true;

                    break;
                }
            }
            return !hasPointInside;
        }

        protected static bool IsPointInTriangle(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p)
        {
            bool isWithinTriangle = false;

            //Based on Barycentric coordinates
            float denominator = ((p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y));

            float a = ((p2.y - p3.y) * (p.x - p3.x) + (p3.x - p2.x) * (p.y - p3.y)) / denominator;
            float b = ((p3.y - p1.y) * (p.x - p3.x) + (p1.x - p3.x) * (p.y - p3.y)) / denominator;
            float c = 1 - a - b;

            //The point is within the triangle
            if (a > 0f && a < 1f && b > 0f && b < 1f && c > 0f && c < 1f)
            {
                isWithinTriangle = true;
            }

            return isWithinTriangle;
        }

        protected bool FindHole(HalfEdgedNode startNode)
        {
            if (startNode.Edges.Count == 0) return false; //모든 연결된 구멍을 찾았음
            List<HalfEdgedNode> newPolygon = new List<HalfEdgedNode>();
            Dictionary<HalfEdgedNode, HalfEdgedEdge> holeNodeEdges = new Dictionary<HalfEdgedNode, HalfEdgedEdge>();
            HalfEdgedNode currentNode = startNode;
            newPolygon.Add(currentNode);
            HalfEdgedEdge currentEdge = startNode.Edges[0];
            HalfEdgedNode nextNode = currentEdge.NodeEnd;
            holeNodeEdges.Add(currentNode, currentEdge);
            while(nextNode != startNode) //한 바퀴 돌아 startNode로 돌아올 때까지
            {
                if (nextNode.MostRightEdge(currentEdge) == null) break;
                currentNode = nextNode;
                currentEdge = nextNode.MostRightEdge(currentEdge);
                newPolygon.Add(currentNode);
                holeNodeEdges.Add(currentNode, currentEdge);
                nextNode = currentEdge.NodeEnd;
            }
            if(nextNode == startNode) //한 바퀴 돌았을 경우 새 구멍 추가
            {
                polygons.Add(newPolygon);
            }
            foreach (KeyValuePair<HalfEdgedNode, HalfEdgedEdge> item in holeNodeEdges) //방문한 간선 모두 제거(한 바퀴를 돌지 못했어도)
            {
                item.Key.Edges.Remove(item.Value);
            }
            return true;
        }
    }
}
