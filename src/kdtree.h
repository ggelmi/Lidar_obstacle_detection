// implementation of the  kd tree

// Structure to represent node of kd tree
//A KD-Tree is a binary tree that splits points between alternating axes. 
//By separating space by splitting regions, nearest neighbor search can be made much faster when using an algorithm like euclidean clustering. 

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


	void insertHelper(Node* & root,uint depth, std::vector<float> point, int id ){
		if(root ==  NULL){
			root = new Node(point,id);
		}else{

			uint c = depth%3;

			if(point[c] < root->point[c]){
				
				insertHelper(root->left, depth+1,point,id);

			}else{
				insertHelper(root->right,depth+1,point,id);

			}
			
		}

	}

	// inserting a new point into the tree
	// the function creates a new node and places correctly with in the root 
	void insert(std::vector<float> point, int id)
	{
		
		
		insertHelper(root,0,point, id);

	}

	void searchHelper(Node* node,std::vector<float> target, uint depth,float distanceTol,std::vector<int>& ids){

		if(node!=NULL){

			// Checking accross boundary

			if((node->point[0]>= (target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol)) && (node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol) ) && (node->point[2]>= (target[2]-distanceTol) && node->point[2]<=(target[2]+distanceTol)) ){

				float  distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - (target[1]))  +  (node->point[2] - target[2])*(node->point[2] - (target[2])));

				if(distance <= distanceTol) {

					ids.push_back(node->id);
				}
				
				

			}

			
			// traversing the kdtree structure using the depth level split alternations

			if((target[depth%3]- distanceTol) < node->point[depth%3]){

				searchHelper(node->left,target,depth+1,distanceTol,ids);
			}
				

			if((target[depth%3]+ distanceTol) > node->point[depth%3]){

				searchHelper(node->right,target,depth+1,distanceTol,ids);
			}
				
		}	


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(root,target,0,distanceTol,ids);
		
		return ids;
	}
	

};




