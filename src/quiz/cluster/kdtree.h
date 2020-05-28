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
	{
//		std::cout<<"new node id = "<<id<<std::endl;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		if (root == nullptr){
//			std::cout<<"creating root node\n";
			root = new Node(point,id);
			return;
		}

		Node * temp = root;
		Node * newNode = new Node(point,id);
		int dims = point.size();
		if (dims>=4)
			dims = 3;
		int depth = 0;
		while(temp != nullptr){
			depth%=dims;
			if (newNode->point[depth] > temp->point[depth] ){
				if (temp->right == nullptr){
					temp->right= newNode;
					return;
				}
				temp = temp->right;
				depth++;
			}
			else{
				if (temp->left == nullptr){
					temp->left=newNode;
					return;
				}
				temp = temp->left;
				depth++;
			}
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
/***************************************************************************************************/
		int dims = target.size();
		if (dims>=4)
			dims = 3;

		Node * temp = root ;
		search_subtree(temp,target,distanceTol,ids,0);





/***************************************************************************************************/
		return ids;
	}

	void search_subtree (Node * temp, std::vector<float> target , float distanceTol, std::vector<int> &ids , int depth){
		if (temp==nullptr)
			return;
		//check point proximity
		bool inRange = check_in_range(target,temp->point,distanceTol);
		if (inRange){
			ids.push_back(temp->id);
		}
		int dims = target.size(); 
		if (dims>=4)
			dims = 3;
		depth%=(dims);
		//check whether point in range of points from both branches
		float levelD = abs( target[depth] - temp->point[depth] );
		if (levelD < distanceTol){
			search_subtree(temp->left,target,distanceTol,ids,depth+1);
			search_subtree(temp->right,target,distanceTol,ids,depth+1);
		}
		// check which branch has the new point
		else if (target[depth] > temp->point[depth]){
			search_subtree(temp->right,target,distanceTol,ids,depth+1);
		}
		else{
			search_subtree(temp->left,target,distanceTol,ids,depth+1);
		}

	}


	bool check_in_range(std::vector<float> target , std::vector<float> checkNodePoint,float distanceTol){
		int dims = (target.size());
		if (dims>=4)
			dims = 3;
		float dist=0;
		// loop on all dimensions of a point to check proximity
		for (int i =0; i <dims ; i++){
			// for euclidean distance
			dist += (target[i]-checkNodePoint[i])*(target[i]-checkNodePoint[i]);
			// for distance tolerance per single axis
//			if ( abs( target[i]-checkNodePoint[i] ) > distanceTol )
//				return false;
		}
		// for euclidean distance
		dist = sqrt(dist);
		if (dist>distanceTol)
			return false;

		return true;
	}
	

};




