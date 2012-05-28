var Mathematics = require("goom-math"), BoundingSphere = require("./bounding_sphere");

/**
	Creates a new BoundingVolumeHierarchyNode.
	@class This class is used in the BV hirarchies to build the hierarchies.
	@exports BoundingVolumeHierarchyNode as Physics.BoundingVolumeHierarchyNode
	@property {Physics.BoundingVolumeHierarchyNode} parent This node's parent.
	@params {Physics.BoundingSphere} volume A bounding volume for the new node.
	@params {Physics.RigidBody} body A rigid body to be stored in the new node.
	@property {Array} children BoundingVolumeHierarchyNode array holding two children.
	@property {Physics.BoundingSphere} volume The bounding volume class.
	@property {Physics.RigidBody} body If this is a leaf node, it hols the body in this node.
	@property {Physics.BoundingVolumeHierarchyNode} parent This node's parent.
*/
function BoundingVolumeHierarchyNode(parent, body, volume) {
	this.parent = parent !== null && parent !== undefined? parent : null;
	this.body = body !== null && body !== undefined? body : null;
	this.volume = volume !== null && volume !== undefined? volume : new BoundingSphere();
	if (this.body !== null && this.body !== undefined) this.body.boundingVolumeHierarchyNode = this;
	this.children = new Array(2);
}

/**
	Checks wether the node is at the bottom of the hierarchy. If it is a leaf,
	it will hold a body.
	@returns {Boolean} true if the node is a leaf node, false otherwise.
*/
BoundingVolumeHierarchyNode.prototype.isLeaf = function() {
	return this.body !== null && this.body !== undefined;
};

/**
	Checks wether the node is at the top of the hierarchy. If it is not a root node,
	it will hold a parent node.
	@returns {Boolean} true if the node is a root node, false otherwise.
*/
BoundingVolumeHierarchyNode.prototype.isRoot = function() {
	return this.parent === null || this.parent === undefined;
};

/**
	Checks whether a node is overlapping another.
	@params {Physics.BoundingVolumeHierarchyNode} The other BoundingVolumeHierarchyNode to
	check against.
	@returns {Boolean} true if node is overlapping the other node, false otherwise.
*/
BoundingVolumeHierarchyNode.prototype.overlaps = function(other) {
	return this.volume.overlaps(other.volume);
};

/**
	Gets an array with the potential contacts between the node's children upto the given
	limit.
	@params {Array} contacts Array holding the possible contacts, where the newly found possible contacts will be pushed.
	@params {Number} [limit=1000] The maximum number of possible contacts.
	@returns {Number} The ammount of newly found contacts.
*/
BoundingVolumeHierarchyNode.prototype.potentialContacts = function(contacts, limit) {
	if (limit === null || limit === undefined) limit = 1000;
	if (this.isLeaf() || limit === 0) return 0;

	return this.children[0].potentialContactsWith(this.children[1], contacts, limit);
};

/**
	Gets an array with the potential contacts between this node and the given node, upto
	the given limit.
	@params {Physics.BoundingVolumeHierarchyNode} other The node to check against.
	@params {Array} contacts Array holding the possible contacts, where the newly found possible contacts will be pushed.
	@params {Number} limit The maximum number of possible contacts
	@returns {Number} The ammount of newly found contacts.
*/
BoundingVolumeHierarchyNode.prototype.potentialContactsWith = function(other, contacts, limit) {
	//No contacts if the bonunding volumes are not overlapping or no more of them fit.
	if (!this.overlaps(other) || limit === 0) return 0;
	//If both nodes are leaf nodes, their bodies are potentially in contact.
	if (this.isLeaf() && other.isLeaf()) {
		contacts.push([this.body, other.body]);
		//TODO: There has to be a better way to store this than creating an array per contact.
		return 1;
	}

	var num_contacts = 0;
	if (other.isLeaf() || (!this.isLeaf() && (this.volume.size() >= other.volume.size()))) {
		//Recurse into the first child.
		num_contacts = this.children[0].potentialContactsWith(other, contacts, limit);
		//If there is enough space for contacts, recurse into the other child.
		if (limit > num_contacts) {
			num_contacts += this.children[1].potentialContactsWith(other, contacts, limit - num_contacts);
		}
	} else {
		//Recurse the other node's first child.
		num_contacts = this.potentialContactsWith(other.children[0], contacts, limit);
		//If there is enough space for contacts, recurse the other child.
		if (limit > num_contacts) {
			num_contacts += this.potentialContactsWith(other.children[1], contacts, limit - num_contacts);
		}
	}
	return num_contacts;
};

/**
	Calculates the size of the bounding volume to hold its children.
*/
BoundingVolumeHierarchyNode.prototype.calculateBoundingVolume = function() {
	if (!this.isLeaf) {
		this.volume.fitSpheres(this.children[0].volume, this.children[1].volume);
	}
};

/**
	Inserts a new volume and body into the bounding volume hierarchy.
	@params {Physics.RigidBody} body A rigid body to be stored in the new node.
	@params {Physics.BoundingVolume} volume A bounding volume for the new node.
*/
BoundingVolumeHierarchyNode.prototype.insert = function(body, volume) {
	//If this is a leaf node, insert the body and volume as a child node, and move this node down as a child.
	if (this.isLeaf()) {
		//TODO: Can we avoid creating these?
		this.children[0] = new BoundingVolumeHierarchyNode(this, this.body, new BoundingSphere(this.volume.position, this.volume.radious));
		this.children[1] = new BoundingVolumeHierarchyNode(this, body, volume);
		this.body = null;
		this.calculateBoundingVolume();
		return;
	}

	//Otherwise, we need to calculate which child keeps the body.
	if (this.children[0].volume.growth(volume) < this.children[1].volume.growth(volume)) {
		this.children[0].insert(body, volume);
	} else {
		this.children[1].insert(body, volume);
	}
};

/**
	Frees this node, depending on the second value all the nodes down the tree will be dereferenced (which should be freed from memory when the garbage collector kicks in).
	@param {Boolean} [reestructure_tree=true] Wether the tree should be automatically reestrucutred or not, only applies when the node is
	not a root node.
*/
BoundingVolumeHierarchyNode.prototype.free = function(reestructure_tree) {
	if (reestructure_tree === null || reestructure_tree === undefined) reestructure_tree = true;

	if (!this.isRoot() && reestructure_tree) {
		//Get this node's sibling for tree reestructure.
		var sibling = this.parent.children[0] === this ? this.parent.children[1] : this.parent.children[0];
		//Write the sibling's data to into the parent.
		this.parent.volume = sibling.volume;
		this.parent.body = sibling.body;

		if (this.parent.body !== null && this.parent.body !== undefined) {
			this.parent.body.boundingVolumeHierarchyNode = this.parent;
		}

		if (sibling.children[0] !== null && sibling.children[0] !== undefined) {
			this.parent.children[0] = sibling.children[0];
			this.parent.children[0].parent = this.parent;
		} else
			this.parent.children[0] = null;

		if (sibling.children[1] !== null && sibling.children[1] !== undefined) {
			this.parent.children[1] = sibling.children[1];
			this.parent.children[1].parent = this.parent;
		} else
			this.parent.children[1] = null;
	}

	//The following code is not necessary when the node has no children
	if (this.isLeaf()) return;
	//Freeing the children
	this.children[0].free(false);
	this.children[1].free(false);
	this.children[0] = null;
	this.children[1] = null;
};

/**
	This method should be called when the position of the bounding volume has changed, it will move the node to the
	proper level if it is necessary.
*/
BoundingVolumeHierarchyNode.prototype.updateHierarchy = function() {
	//Update the volume's position
	this.body.position.clone(this.volume.position);

	if (!this.isRoot()) {
		//Remove this node from the hierarchy
		this.free();
		//Attempt reinsertion
		this.parent.reinsert(this.body, this.volume);
	}
};

/**
	Attempts to insert a node back into the hierarchy from one of the lower ends.
	@params {Physics.RigidBody} body A rigid body to be stored in the new node.
	@params {Physics.BoundingVolume} volume A bounding volume for the new node.
*/
BoundingVolumeHierarchyNode.prototype.reinsert = function(body, volume) {
	if ((this.volume.growth(volume) <= 0) || this.isRoot()) {
		//Found the insertion point!
		this.insert(body, volume);
	} else {
		//Keep looking up the tree
		this.parent.reinsert(body, volume);
	}
};

module.exports =  BoundingVolumeHierarchyNode;