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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(root, point, id, 0);
	}

	void insertHelper(Node *&node, std::vector<float> point, int id, int depth)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			int cd = depth % point.size(); // Calculate current dimension to split on
			if (point[cd] < node->point[cd])
			{
				insertHelper(node->left, point, id, depth + 1);
			}
			else
			{
				insertHelper(node->right, point, id, depth + 1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}

	void searchHelper(Node *node, std::vector<float> target, float distanceTol, int depth, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			// Check if the current node's point is within the range
			bool withinRange = true;
			for (int i = 0; i < target.size(); ++i)
			{
				if (fabs(target[i] - node->point[i]) > distanceTol)
				{
					withinRange = false;
					break;
				}
			}

			if (withinRange)
			{
				// Calculate Euclidean distance between the target and the current point
				float distance = 0;
				for (int i = 0; i < target.size(); ++i)
				{
					distance += (target[i] - node->point[i]) * (target[i] - node->point[i]);
				}
				distance = sqrt(distance);

				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			// Recursively search the left and right subtrees
			int cd = depth % target.size();
			if (target[cd] - distanceTol < node->point[cd])
			{
				searchHelper(node->left, target, distanceTol, depth + 1, ids);
			}
			if (target[cd] + distanceTol > node->point[cd])
			{
				searchHelper(node->right, target, distanceTol, depth + 1, ids);
			}
		}
	}
};




