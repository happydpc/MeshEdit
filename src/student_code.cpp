#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
	void BezierCurve::evaluateStep()
	{
		// TODO Part 1.
		// Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
		// Store all of the intermediate control points into the 2D vector evaluatedLevels.
		vector<Vector2D> newlevel = std::vector<Vector2D>(); //get the current number of control points
		vector<Vector2D> recentlevel = evaluatedLevels[evaluatedLevels.size() - 1]; //most recent level
		for (int i = 0; i < recentlevel.size() - 1; i++) { //size of new level is the most recent level size minus 1 
			newlevel.push_back((1 - t) * recentlevel[i] + t * recentlevel[i + 1]);
		}
		evaluatedLevels.push_back(newlevel);
	}


	Vector3D BezierPatch::evaluate(double u, double v) const
	{
		// TODO Part 2.
		// Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
		// (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
		// should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)

		std::vector<Vector3D> upoints = std::vector<Vector3D>();
		for (vector<Vector3D> controls : controlPoints) {
			upoints.push_back(evaluate1D(controls, u)); 
		}
		return evaluate1D(upoints, v);
	}

	Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
	{
		// TODO Part 2.
		// Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
		// Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
		//return Vector3D();
		unsigned long i = points.size();
		std::vector<Vector3D> priorstep = points;
		std::vector<Vector3D> nextstep = std::vector<Vector3D>();
		double onemint = 1 - t;
		while (i != 1) {
			nextstep = std::vector<Vector3D>();
			for (int j = 0; j < priorstep.size() - 1; j++) {
				nextstep.push_back(onemint * priorstep[j] + t * priorstep[j + 1]);
			}
			priorstep = nextstep;
			i = priorstep.size();
		}
		return priorstep[0];
	}



	Vector3D Vertex::normal(void) const
	{
		// TODO Part 3.
		// TODO Returns an approximate unit normal at this vertex, computed by
		// TODO taking the area-weighted average of the normals of neighboring
		// TODO triangles, then normalizing.
		//return Vector3D();
		  //using the halfEdgeMesh.cpp's normal function as a template. 
		Vector3D n = Vector3D(0, 0, 0);
		HalfedgeCIter h = halfedge();
		h = h->twin();
		HalfedgeCIter h_orig = h;
		do {
			Vector3D edgeA = (h->next()->vertex()->position - h->vertex()->position);
			Vector3D edgeB = (h->next()->next()->vertex()->position - h->next()->vertex()->position);
			n += cross(edgeA, edgeB); 
			h = h->next()->twin();
		} while (h != h_orig);
		return n.unit();
	}

	EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
	{
		// TODO Part 4.
		// TODO This method should flip the given edge and return an iterator to the flipped edge.
		//return e0
		  //set up got fom http://15462.courses.cs.cmu.edu/fall2015content/misc/HalfedgeEdgeOpImplementationGuide.pdf. link provided by TA on piazza
		if (!e0->halfedge()->face()->isBoundary() && !e0->halfedge()->twin()->face()->isBoundary()) {
			HalfedgeIter h0 = e0->halfedge();
			HalfedgeIter h1 = h0->next();
			HalfedgeIter h2 = h1->next();
			HalfedgeIter h3 = h0->twin();
			HalfedgeIter h4 = h3->next();
			HalfedgeIter h5 = h4->next();
			HalfedgeIter h6 = h1->twin();
			HalfedgeIter h7 = h2->twin();
			HalfedgeIter h8 = h4->twin();
			HalfedgeIter h9 = h5->twin();

			VertexIter v0 = h0->vertex();
			VertexIter v1 = h3->vertex();
			VertexIter v2 = h2->vertex();
			VertexIter v3 = h5->vertex();

			EdgeIter e1 = h1->edge();
			EdgeIter e2 = h2->edge();
			EdgeIter e3 = h4->edge();
			EdgeIter e4 = h5->edge();

			FaceIter f0 = h0->face();
			FaceIter f1 = h3->face();

			h0->setNeighbors(h1, h3, v3, e0, f0); //directions based on CMU source
			h1->setNeighbors(h2, h7, v2, e2, f0);
			h2->setNeighbors(h0, h8, v0, e3, f0);
			h3->setNeighbors(h4, h0, v2, e0, f1);
			h4->setNeighbors(h5, h9, v3, e4, f1);
			h5->setNeighbors(h3, h6, v1, e1, f1);

			h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
			h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
			h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
			h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());
			f0->halfedge() = h0;
			f1->halfedge() = h3;
			e0->halfedge() = h0;
			e1->halfedge() = h5;
			e2->halfedge() = h1;
			e3->halfedge() = h2;
			e4->halfedge() = h4;
			v0->halfedge() = h2;
			v1->halfedge() = h5;
			v2->halfedge() = h3;
			v3->halfedge() = h0;

			return e0;
		}

	}

	VertexIter HalfedgeMesh::splitEdge(EdgeIter e0)
	{
		// TODO Part 5.
		// TODO This method should split the given edge and return an iterator to the newly inserted vertex.
		// TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
		//return newVertex();

		if (!e0->halfedge()->face()->isBoundary() && !e0->halfedge()->twin()->face()->isBoundary()) {
			// before
			HalfedgeIter h0 = e0->halfedge();
			HalfedgeIter h1 = h0->next();
			HalfedgeIter h2 = h1->next();
			HalfedgeIter h3 = h0->twin();
			HalfedgeIter h4 = h3->next();
			HalfedgeIter h5 = h4->next();
			HalfedgeIter h6 = h1->twin();
			HalfedgeIter h7 = h2->twin();
			HalfedgeIter h8 = h4->twin();
			HalfedgeIter h9 = h5->twin();

			VertexIter v0 = h0->vertex();
			VertexIter v1 = h3->vertex();3
			VertexIter v2 = h2->vertex();
			VertexIter v3 = h5->vertex();

			EdgeIter e1 = h1->edge();
			EdgeIter e2 = h2->edge();
			EdgeIter e3 = h4->edge();
			EdgeIter e4 = h5->edge();

			FaceIter f0 = h0->face();
			FaceIter f1 = h3->face();

	
			Vector3D i = e0->halfedge()->vertex()->position;
			Vector3D j = e0->halfedge()->twin()->vertex()->position;
			Vector3D new_vertex = (i + j) / 2;
			VertexIter mid = this->newVertex();
			mid->isNew = true;
			mid->position = new_vertex;
			//new elements
			EdgeIter e0above = this->newEdge();
			EdgeIter e0left = this->newEdge();
			EdgeIter e0right = this->newEdge();
			e0left->isNew = true;
			e0right->isNew = true;
			HalfedgeIter h0twin = this->newHalfedge();
			HalfedgeIter h0right = this->newHalfedge();
			HalfedgeIter h1next = this->newHalfedge();
			HalfedgeIter h3twin = this->newHalfedge();
			HalfedgeIter h3next = this->newHalfedge();
			HalfedgeIter h4next = this->newHalfedge();
			FaceIter f2 = this->newFace();
			FaceIter f3 = this->newFace();

			h0->setNeighbors(h0right, h0twin, v0, e0, f0);
			h0right->setNeighbors(h2, h1next, mid, e0right, f0);//new
			h0twin->setNeighbors(h4, h0, mid, e0, f3); //new
			h1->setNeighbors(h1next, h6, v1, e1, f2);
			h1next->setNeighbors(h3twin, h0right, v2, e0right, f2);//new
			h2->setNeighbors(h0, h7, v2, e2, f0);
			h3->setNeighbors(h3next, h3twin, v1, e0above, f1);
			h3next->setNeighbors(h5, h4next, mid, e0left, f1);//new
			h3twin->setNeighbors(h1, h3, mid, e0above, f2); //new
			h4->setNeighbors(h4next, h8, v0, e3, f3);
			h4next->setNeighbors(h0twin, h3next, v3, e0left, f3);//new
			h5->setNeighbors(h3, h9, v3, e4, f1);
			h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
			h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());
			h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());
			h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());

			f0->halfedge() = h0;
			f1->halfedge() = h3;
			f2->halfedge() = h3twin;
			f3->halfedge() = h0twin;

			v0->halfedge() = h0;
			v1->halfedge() = h3;
			v2->halfedge() = h1next;
			v3->halfedge() = h4next;

			mid->halfedge() = h0twin;

			e0->halfedge() = h0;
			e0above->halfedge() = h3;
			e0left->halfedge() = h4next;
			e0right->halfedge() = h1next;
			e1->halfedge() = h1;
			e2->halfedge() = h2;
			e3->halfedge() = h4;
			e4->halfedge() = h5;




			return mid;
		}
	}



	void MeshResampler::upsample(HalfedgeMesh& mesh)
	{
		// TODO Part 6.
		// This routine should increase the number of triangles in the mesh using Loop subdivision.
		// Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
		// Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
		// using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
		// the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
		// the new mesh based on the values we computed for the original mesh.


		// TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
		// TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
		// TODO a vertex of the original mesh.


		// TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


		// TODO Next, we're going to split every edge in the mesh, in any order.  For future
		// TODO reference, we're also going to store some information about which subdivided
		// TODO edges come from splitting an edge in the original mesh, and which edges are new,
		// TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
		// TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
		// TODO just split (and the loop will never end!)


		// TODO Now flip any new edge that connects an old and new vertex.


		// TODO Finally, copy the new vertex positions into final Vertex::position.

	   //return;




	


		HalfedgeIter halfs;
		Vector3D neighbor_position_sum;
		for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
			v->isNew = false;
			double u;
			neighbor_position_sum = Vector3D(0, 0, 0);

			int neighbor = 0;
			halfs = v->halfedge();
			do {
				neighbor += 1;
				neighbor_position_sum += halfs->twin()->vertex()->position;
				halfs = halfs->twin()->next();
			} while (halfs != v->halfedge());
			if (neighbor == 3) {
				u = (3.0f / 16);
				v->newPosition = ((1 - neighbor * u) * v->position + u * neighbor_position_sum);
			}
			else {
				u = 3.0f / (8 * neighbor);
				v->newPosition = ((1 - neighbor * u) * v->position + u * neighbor_position_sum);
			}

		}

		Vector3D a = Vector3D(0,0,0);
		Vector3D b = Vector3D(0, 0, 0);
		Vector3D c = Vector3D(0, 0, 0);
		Vector3D d = Vector3D(0, 0, 0);

		for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
			e->isNew = false;
			halfs = e->halfedge();
			a = halfs->vertex()->position;
			c = halfs->twin()->vertex()->position;
			b = halfs->next()->next()->vertex()->position;
			d = halfs->twin()->next()->next()->vertex()->position;
			e->newPosition = (a + c) * 3/8.0f + (b + d)*1/8.0f;  
		}

		

		VertexIter v;	
		Vector3D storenewpos;
		storenewpos = Vector3D(0, 0, 0);
		for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
			bool bothnotnew = !e->halfedge()->vertex()->isNew && !e->halfedge()->twin()->vertex()->isNew;
			if (bothnotnew) {
				storenewpos = e->newPosition;
				VertexIter new_vertex = mesh.splitEdge(e);
				new_vertex->newPosition = storenewpos;  
			}

		}

	
		for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
			if (e->isNew) {
				halfs = e->halfedge();
				
				bool newold = halfs->vertex()->isNew && !halfs->twin()->vertex()->isNew;
				bool oldnew = !halfs->vertex()->isNew && halfs->twin()->vertex()->isNew;
				if (newold || oldnew) {
					mesh.flipEdge(e);
				}
			}
		}

		for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
			v->position = v->newPosition;
		}

	}
}
