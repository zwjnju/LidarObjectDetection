/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** ptr, int depth, std::vector<float> point, int id) {
		if((*ptr) == NULL) {
			*ptr = new Node(point, id);
		}else {
			uint idx = depth % 2;
			if((*ptr)->point[idx] > point[idx]) {
				insertHelper(&((*ptr)->left), depth+1, point, id);
			}else {
				insertHelper(&((*ptr)->right), depth+1, point, id);
			}
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	void searchHelper(Node* node, int depth, std::vector<float> target, float distanceTol, std::vector<int>& ids) {
		if(node == NULL) {
			return;
		}else {
			if(fabs(target[0] - node->point[0]) < distanceTol && fabs(target[1] - node->point[1]) < distanceTol) {
				float distance = sqrt(pow(target[0] - node->point[0], 2) + pow(target[1] - node->point[1], 2));
				if(distance < distanceTol) {
					ids.push_back(node->id);
				}				
			}
			int dep = depth % 2;
			if(target[dep] - distanceTol < node->point[dep]) {
				searchHelper(node->left, depth+1, target, distanceTol, ids);
			}
			if(target[dep] + distanceTol > node->point[dep]) {
				searchHelper(node->right, depth+1, target, distanceTol, ids);
			} 
		}
	}
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}
	

};




