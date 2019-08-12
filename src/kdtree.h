/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <math.h>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};


template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node<PointT>** ptr, int depth, PointT point, int id) {

		if((*ptr) == NULL) {
			*ptr = new Node<PointT>(point, id);
		}else {
			uint idx = depth % 3;
			if(idx == 0) {
				if((*ptr)->point.x > point.x) {
					insertHelper(&((*ptr)->left), depth+1, point, id);
				} else {
					insertHelper(&((*ptr)->right), depth+1, point, id);
				}
			} else if(idx == 1) {
				if((*ptr)->point.y > point.y) {
					insertHelper(&((*ptr)->left), depth+1, point, id);
				} else {
					insertHelper(&((*ptr)->right), depth+1, point, id);
				}
			} else {
				if((*ptr)->point.z > point.z) {
					insertHelper(&((*ptr)->left), depth+1, point, id);
				} else {
					insertHelper(&((*ptr)->right), depth+1, point, id);
				}
			}
		}
		
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	void searchHelper(Node<PointT>* node, int depth, PointT target, float distanceTol, std::vector<int>& ids) {
		if(node == NULL) {
			return;
		}else {
			if(fabs(target.x - node->point.x) < distanceTol && fabs(target.y - node->point.y) < distanceTol && fabs(target.z - node->point.z) < distanceTol) {
				float distance = sqrt(pow(target.x - node->point.x, 2) + pow(target.y - node->point.y, 2) + pow(target.z - node->point.z, 2));
				if(distance < distanceTol) {
					ids.push_back(node->id);
				}				
			}
			int dep = depth % 3;
			if(dep == 0) {
				if(target.x - distanceTol < node->point.x) {
					searchHelper(node->left, depth+1, target, distanceTol, ids);
				}
				if(target.x + distanceTol > node->point.x) {
					searchHelper(node->right, depth+1, target, distanceTol, ids);
				}
			} else if(dep == 1) {
				if(target.y - distanceTol < node->point.y) {
					searchHelper(node->left, depth+1, target, distanceTol, ids);
				}
				if(target.y + distanceTol > node->point.y) {
					searchHelper(node->right, depth+1, target, distanceTol, ids);
				}
			} else {
				if(target.z - distanceTol < node->point.z) {
					searchHelper(node->left, depth+1, target, distanceTol, ids);
				}
				if(target.z + distanceTol > node->point.z) {
					searchHelper(node->right, depth+1, target, distanceTol, ids);
				}
			}
		}
	}
	std::vector<int> search(PointT target, float distanceTol)
	{
		
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}
	

};




