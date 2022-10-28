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
                Vector3 vec0 = new Vector3(polygon[0].X, 0, polygon[0].Y);
                Vector3 vec1 = new Vector3(polygon[1].X, 0, polygon[1].Y);
                Vector3 vec2 = new Vector3(polygon[2].X, 0, polygon[2].Y);
                Vector3 vec3 = new Vector3(polygon[3].X, 0, polygon[3].Y);
                vertices.Add(vec0);
                vertices.Add(vec1);
                vertices.Add(vec2);
                vertices.Add(vec3);
                triangles.Add(vertices.IndexOf(vec0));
                triangles.Add(vertices.IndexOf(vec1));
                triangles.Add(vertices.IndexOf(vec3));
                triangles.Add(vertices.IndexOf(vec1));
                triangles.Add(vertices.IndexOf(vec2));
                triangles.Add(vertices.IndexOf(vec3));
            }
            IMesh.vertices = vertices.ToArray();
            IMesh.triangles = triangles.ToArray();
            return IMesh;
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
