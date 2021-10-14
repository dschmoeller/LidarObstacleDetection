/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	void insertHelp(Node** node, std::vector<float> point, int id, int depth){
		if(*node == NULL){
			*node = new Node(point, id); 
		}
		else{
			// Use depth as index to directy refer to the x or y value
			int idx = depth % 3; 
			if((*node)->point[idx] > point[idx]){
				// Branch to left node
				insertHelp(&((*node)->left), point, id, depth+1); 
			}
			else{
				// Branch to the right
				insertHelp(&((*node)->right), point, id, depth+1); 
			}
		}
	}

	
	void insert(std::vector<float> point, int id)
	{
		// Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		int depth = 0; // 0 -> Split x axis
		insertHelp(&root, point, id, depth); 
	}

	void searchHelper(std::vector<float> target, float distanceTol, Node* node, int depth, std::vector<int>& ids){
		if(node!= NULL){ 
			// Is node within target region? 
			if (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) &&
				node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol) &&
				node->point[2] >= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol)){
				// Measure eucledian distance and add point to list if distance is small enogh
				float distX = node->point[0] - target[0]; 
				float distY = node->point[1] - target[1]; 
				float distZ = node->point[2] - target[2]; 
				float dist = sqrt(distX*distX + distY*distY + distZ*distZ); 
				if(dist <= distanceTol){
					ids.push_back(node->id); 
				}			
			}
			// Branch off search region by only continuing search if target box lies within KD split region
			if ((target[depth%3] - distanceTol) < node->point[depth%3]){
				// Keep searching on "left" side
				searchHelper(target, distanceTol, node->left, depth+1, ids); 
			} 	
			if ((target[depth%3] + distanceTol) > node->point[depth%3]){
				// Keep searching on the "right" side
				searchHelper(target, distanceTol, node->right, depth+1, ids); 
			} 
		}	
	}
	
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, root, 0, ids); 
		
		return ids;
	}	
};




