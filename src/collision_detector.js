if (typeof define !== 'function') {
	var define = require('amdefine')(module);
}

define(["goom-math", "./contact", "./intersection_tests", "./primitives","./rigid_body"], function(Mathematics, Contact, IntersectionTests, Primitives) {
	/**
		Creates a new CollisionDetector.
		@class Collision detectors are used to check collisions between primitive and fetch the contact data.
		@param {Number} cache_limit The max size of the cache for contacts, cache_limit contacts will be created
			when the constructor is called, and they will be reused throught time. When the contact number is over
			the limit, new contacts will be instantiated but not cached.
		@property {Number} MAX_CONTACTS The max size of the cache for contacts.
		@property {Array} contactCache An array holding the cached contacts.

		@exports CollisionDetector as Physics.CollisionDetector
	*/
	var CollisionDetector = (function() {
		function CollisionDetector(cache_limit) {
			this.MAX_CONTACTS = cache_limit === null || cache_limit === undefined ? 30: cache_limit;
			this.contactCache = new Array(this.MAX_CONTACTS);
			this.contactCacheTop = 0;
			//Initialize the contact cache.
			for (var i = this.MAX_CONTACTS; i >= 0; i--) {
				this.contactCache[i] = new Contact();
			}
			//Helper data used to avoid instantiating objects at runtime.
			this.__helperVector = new Mathematics.Vector3D();
			this.__helperVector2 = new Mathematics.Vector3D();
			this.__helperVector3 = new Mathematics.Vector3D();
			this.__helperVector4 = new Mathematics.Vector3D();
			this.__helperVector5 = new Mathematics.Vector3D();
			this.__helperVector6 = new Mathematics.Vector3D();
			this.__helperVector7 = new Mathematics.Vector3D();

			this.__helperMatrix = new Mathematics.Matrix3D();
			this.__axisTestData = new Array(3);
		}

		//We will need to go through each combination of vectors, and it's best not to create this on the fly.
		CollisionDetector.multipliers = [[1, 1, 1], [-1, 1, 1], [1, -1, 1], [-1, -1, 1], [1, 1, -1], [-1, 1, -1], [1, -1, -1], [-1, -1, -1]];

		/**
			Returns the last free contact in the cache if there is any, and creates a new contact when there isn't.
			@inner
		*/
		CollisionDetector.prototype.__getFreeContact = function() {
			//When there is no contacts free, instantiate a new one.
			if (this.contactCacheTop >= this.MAX_CONTACTS) return new Contact();
			//Return the last empty contact from cache.
			this.contactCacheTop++;
			return this.contactCache[this.contactCacheTop - 1];
		};

		/**
			Sets the contact cache to the initial status, reusing the cached contacts.
		*/
		CollisionDetector.prototype.clearCache = function() {
			this.contactCacheTop = 0;
		};

		/**
			Checks for contacts between two spheres.
			@param {Physics.Sphere} sphere_one The first sphere to check.
			@param {Physics.Sphere} sphere_two The second sphere to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.sphereAndSphere = function(sphere_one, sphere_two, data, contacts) {
			//Cache sphere positions
			var position_one = sphere_one.position(this.__helperVector);
			var position_two = sphere_two.position(this.__helperVector2);
			//Find the vector between the objects
			var midline = position_one.substract(position_two, this.__helperVector2);
			var size = midline.magnitude();
			//Check if we are too far away
			if ((size <= 0) || (size >= (sphere_one.radious + sphere_two.radious))) return false;
			
			//Prepare the contact.
			var contact = this.__getFreeContact();
			//Manually create the normal
			midline.scale(1 / size, contact.normal);
			position_one.substract(midline.scale(0.5), contact.point);
			contact.penetration = sphere_one.radious + sphere_two.radious - size;
			contact.setContactData(sphere_one.body, sphere_two.body, data.restitution, data.friction);
			contacts.push(contact);
			return true;
		};

		/**
			Checks for contacts between a sphere and a half-plane(a plane with infinite solid behind de plane).
			@param {Physics.Sphere} sphere The sphere to check.
			@param {Physics.Plane} plane The plane to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.sphereAndHalfSpace = function(sphere, plane, data, contacts) {
			var sphere_position = sphere.position(this.__helperVector);
			var distance = plane.normal.dotProduct(sphere_position) - sphere.radious - plane.offset;
			//Check if we are too far away
			if (distance >= 0) return false;
			//Prepare the contact.
			var contact = this.__getFreeContact();
			sphere_position.substract(plane.normal.scale(distance + sphere.radious, contact.point), contact.point);
			plane.normal.clone(contact.normal);
			contact.penetration = -distance;
			contact.setContactData(sphere.body, null, data.restitution, data.friction);
			contacts.push(contact);
			return true;
		};

		/**
			Checks for contacts between a box and a half space.
			@param {Physics.Box} box The box to check.
			@param {Physics.Plane} plane The plane to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.boxAndHalfSpace = function(box, plane, data, contacts) {
			//Early-out check for intersection, performance improvement
			if (!IntersectionTests.boxAndHalfSpace(box, plane)) return false;
			
			contacts_found = false;
			//Go through the different vertices
			for (i = 0; i <= 7; i++) {
				var vertex_position = this.__helperVector.set(CollisionDetector.multipliers[i][0], CollisionDetector.multipliers[i][1], CollisionDetector.multipliers[i][2]);
				vertex_position.componentProduct(box.halfSize);
				box.transformationMatrix.transformVector(vertex_position);
				//Calculate the distance from the plane
				var vertex_distance = vertex_position.dotProduct(plane.normal);
				if (vertex_distance <= plane.offset) {
					//Create contact data
					var contact = this.__getFreeContact();
					//The point is halfway between the vertex and the point.
					plane.normal.scale(-(vertex_distance - plane.offset), contact.point);
					contact.point.add(vertex_position);
					plane.normal.clone(contact.normal);
					contact.penetration = plane.offset - vertex_distance;
					contact.setContactData(box.body, null, data.restitution, data.friction);
					contacts.push(contact);
					contacts_found = true;
				}
			}
			return contacts_found;
		};

		/**
			Checks for contacts between a box and a sphere.
			@param {Physics.Box} box The box to check.
			@param {Physics.Sphere} sphere The sphere to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.boxAndSphere = function(box, sphere, data, contacts) {
			//We need the center of the sphere to be in box coordinates
			var sphere_center = sphere.position(this.__helperVector);
			var relative_center = box.transformationMatrix.transformInverseVector(sphere_center, this.__helperMatrix);

			//Early-out check
			if (((Math.abs(relative_center.x) - sphere.radious) > box.halfSize.x) ||
				((Math.abs(relative_center.y) - sphere.radious) > box.halfSize.y) ||
				((Math.abs(relative_center.z) - sphere.radious) > box.halfSize.z)) {

				return false;
			}

			//Clamp each coordinate to the box
			var distance = relative_center.x;
			if (distance > box.halfSize.x) distance = box.halfSize.x;
			if (distance < -box.halfSize.x) distance = -box.halfSize.x;
			this.__helperVector2.x = distance;

			distance = relative_center.y;
			if (distance > box.halfSize.y) distance = box.halfSize.y;
			if (distance < -box.halfSize.y) distance = -box.halfSize.y;
			this.__helperVector2.y = distance;
			
			distance = relative_center.z;
			if (distance > box.halfSize.z) distance = box.halfSize.z;
			if (distance < -box.halfSize.z) distance = -box.halfSize.z;
			this.__helperVector2.z = distance;
			
			distance = this.__helperVector2.substract(relative_center, this.__helperVector).squaredMagnitude();
			if (distance > (sphere.radious * sphere.radious)) return false;
			
			sphere_center = sphere.position(this.__helperVector);
			//Create contact data
			var contact = this.__getFreeContact();
			box.transformationMatrix.transformVector(this.__helperVector2);
			this.__helperVector2.clone(contact.point);
			this.__helperVector2.substract(sphere_center).normalize().clone(contact.normal);
			contact.penetration = sphere.radious - Math.sqrt(distance);
			contact.setContactData(box.body, sphere.body, data.restitution, data.friction);
			contacts.push(contact);
			return true;
		};

		/**
			If the given boxes collide and the penetration on the requested axis is smaller than the previously smallest,
			it updates these values.
			@inner
		*/
		CollisionDetector.prototype.__testAxis = function(box_one, box_two, axis, to_centre, index, smallest_penetration, smallest_case) {
			//Don't check almost parallel axes
			if (axis.squaredMagnitude() < 0.0001) {
				this.__axisTestData[0] = true, this.__axisTestData[1] = smallest_penetration, this.__axisTestData[2] = smallest_case;
				return this.__axisTestData;
			}

			//Get the penetration
			axis.normalize();
			var projection_one = box_one.halfSize.x * Math.abs(axis.dotProduct(box_one.axisVector(0, this.__helperVector))) +
								box_one.halfSize.y * Math.abs(axis.dotProduct(box_one.axisVector(1, this.__helperVector))) +
								box_one.halfSize.z * Math.abs(axis.dotProduct(box_one.axisVector(2, this.__helperVector)));
			var projection_two = box_two.halfSize.x * Math.abs(axis.dotProduct(box_two.axisVector(0, this.__helperVector))) +
								box_two.halfSize.y * Math.abs(axis.dotProduct(box_two.axisVector(1, this.__helperVector))) +
								box_two.halfSize.z * Math.abs(axis.dotProduct(box_two.axisVector(2, this.__helperVector)));
			var penetration = projection_one + projection_two - Math.abs(to_centre.dotProduct(axis));
		
			if (penetration < 0) {
				this.__axisTestData[0] = false, this.__axisTestData[1] = smallest_penetration, this.__axisTestData[2] = smallest_case;
				return this.__axisTestData;
			}

			//Did we find a contact with smaller penetration?
			if (penetration < smallest_penetration) {
				smallest_penetration = penetration;
				smallest_case = index;
			}
			
			this.__axisTestData[0] = true, this.__axisTestData[1] = smallest_penetration, this.__axisTestData[2] = smallest_case;
			return this.__axisTestData;
		};

		axis_index_to_name = function() {
			switch(index) {
				case 0: return 'x';
				case 1: return 'y';
				case 2: return 'z';
			}
		};

		/**
			Checks for contacts between two boxes.
			@param {Physics.Box} box_one The first box to check.
			@param {Physics.Box} box_two The second box to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.boxAndBox = function(box_one, box_two, data, contacts) {
			//Get the vector from the position of the first box to the position of the second box
			var to_centre = box_two.position(this.__helperVector2).substract(box_one.position(this.__helperVector));
			//Assume there is no contact
			var penetration, best, axis_test_data;
			penetration = best = Number.MAX_VALUE;
			var test = false;

			//Test the different axes for intersections keeping track of the smallest penetration
			axis_test_data = this.__testAxis(box_one, box_two, box_one.axisVector(0, this.__helperVector3), to_centre, 0, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_one.axisVector(1, this.__helperVector3), to_centre, 1, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_one.axisVector(2, this.__helperVector3), to_centre, 2, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(0, this.__helperVector3), to_centre, 3, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(1, this.__helperVector3), to_centre, 4, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(2, this.__helperVector3), to_centre, 5, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			//Store the best axis-major
			var best_single_axis = best;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(0, this.__helperVector3).crossProduct(box_one.axisVector(0, this.__helperVector)), to_centre, 6, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(1, this.__helperVector3).crossProduct(box_one.axisVector(0, this.__helperVector)), to_centre, 7, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(2, this.__helperVector3).crossProduct(box_one.axisVector(0, this.__helperVector)), to_centre, 8, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(0, this.__helperVector3).crossProduct(box_one.axisVector(1, this.__helperVector)), to_centre, 9, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(1, this.__helperVector3).crossProduct(box_one.axisVector(1, this.__helperVector)), to_centre, 10, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(2, this.__helperVector3).crossProduct(box_one.axisVector(1, this.__helperVector)), to_centre, 11, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(0, this.__helperVector3).crossProduct(box_one.axisVector(2, this.__helperVector)), to_centre, 12, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(1, this.__helperVector3).crossProduct(box_one.axisVector(2, this.__helperVector)), to_centre, 13, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;
			axis_test_data = this.__testAxis(box_one, box_two, box_two.axisVector(2, this.__helperVector3).crossProduct(box_one.axisVector(2, this.__helperVector)), to_centre, 14, penetration, best), test = axis_test_data[0], penetration = axis_test_data[1], best = axis_test_data[2];
			if (!test) return false;

			var contact;
			if (best < 3) {
				//We've got a vertex of box two on a face of box_one
				//Create contact data
				contact = this.__getFreeContact();
				box_one.axisVector(best, contact.normal);
				if (contact.normal.dotProduct(to_centre) > 0) {
					contact.normal.scale(-1);
				}

				box_two.halfSize.clone(contact.point);
				if (box_two.axisVector(0, this.__helperVector3).dotProduct(contact.normal) < 0) contact.point.x = -contact.point.x;
				if (box_two.axisVector(1, this.__helperVector3).dotProduct(contact.normal) < 0) contact.point.y = -contact.point.y;
				if (box_two.axisVector(2, this.__helperVector3).dotProduct(contact.normal) < 0) contact.point.z = -contact.point.z;
			
				box_two.transformationMatrix.transformVector(contact.point);
				contact.penetration = penetration;
				contact.setContactData(box_one.body, box_two.body, data.restitution, data.friction);
				contacts.push(contact);
				return true;
			} else if (best < 6) {
				//We've got a vertex of box one on a face of two
				best -= 3;
				to_centre.scale(-1);
				//Create contact data
				contact = this.__getFreeContact();
				box_two.axisVector(best, contact.normal);
				if (contact.normal.dotProduct(to_centre) > 0) {
					contact.normal.scale(-1);
				}

				box_one.halfSize.clone(contact.point);
				if (box_one.axisVector(0, this.__helperVector3).dotProduct(contact.normal) < 0) contact.point.x = -contact.point.x;
				if (box_one.axisVector(1, this.__helperVector3).dotProduct(contact.normal) < 0) contact.point.y = -contact.point.y;
				if (box_one.axisVector(2, this.__helperVector3).dotProduct(contact.normal) < 0) contact.point.z = -contact.point.z;
				
				box_one.transformationMatrix.transformVector(contact.point);
				contact.penetration = penetration;
				contact.setContactData(box_two.body, box_one.body, data.restitution, data.friction);
				contacts.push(contact);
				return true;
			} else {
				//Edge-to-edge contact
				best -= 6;
				var box_two_axis_index = best % 3;
				var box_one_axis_index = (best - box_two_axis_index) / 3;
				var box_one_axis = box_one.axisVector(box_one_axis_index, this.__helperVector);
				var box_two_axis = box_two.axisVector(box_two_axis_index, this.__helperVector3);
				var axis = box_two_axis.crossProduct(box_one_axis, this.__helperVector4).normalize();
				//This axis should point from box one to box two
				if (axis.dotProduct(to_centre) > 0) axis.scale(-1);
				//Get the edges
				var point_on_box_one_edge = box_one.halfSize.clone(this.__helperVector5);
				var point_on_box_two_edge = box_two.halfSize.clone(this.__helperVector6);
		
				if (0 === box_one_axis_index) {
					point_on_box_one_edge.x = 0;
				} else if (box_one.axisVector(0, this.__helperVector7).dotProduct(axis) > 0) {
					point_on_box_one_edge.x = -point_on_box_one_edge.x;
				}
				if (0 === box_two_axis_index) {
					point_on_box_two_edge.x = 0;
				} else if (box_two.axisVector(0, this.__helperVector7).dotProduct(axis) < 0) {
					point_on_box_two_edge.x = -point_on_box_two_edge.x;
				}

				if (1 === box_one_axis_index) {
					point_on_box_one_edge.y = 0;
				} else if (box_one.axisVector(1, this.__helperVector7).dotProduct(axis) > 0) {
					point_on_box_one_edge.y = -point_on_box_one_edge.y;
				}
				if (1 === box_two_axis_index) {
					point_on_box_two_edge.y = 0;
				} else if (box_two.axisVector(1, this.__helperVector7).dotProduct(axis) < 0) {
					point_on_box_two_edge.y = -point_on_box_two_edge.y;
				}

				if (2 === box_one_axis_index) {
					point_on_box_one_edge.z = 0;
				} else if (box_one.axisVector(2, this.__helperVector7).dotProduct(axis) > 0) {
					point_on_box_one_edge.z = -point_on_box_one_edge.z;
				}
				if (2 === box_two_axis_index) {
					point_on_box_two_edge.z = 0;
				} else if (box_two.axisVector(2, this.__helperVector7).dotProduct(axis) < 0) {
					point_on_box_two_edge.z = -point_on_box_two_edge.z;
				}
		
				//Move the edges into world coordinates
				box_one.transformationMatrix.transformVector(point_on_box_one_edge);
				box_two.transformationMatrix.transformVector(point_on_box_two_edge);
				//Find out the point of closest approach of the two segments
				var use_one = best_single_axis > 2;
				var sm_one = box_one_axis.squaredMagnitude();
				var sm_two = box_two_axis.squaredMagnitude();
				var dp_one_two = box_two_axis.dotProduct(box_one_axis);
				var to_st = point_on_box_one_edge.substract(point_on_box_two_edge, this.__helperVector7);
				var dp_st_one = box_one_axis.dotProduct(to_st);
				var dp_st_two = box_two_axis.dotProduct(to_st);
				var denominator = sm_one * sm_two - dp_one_two * dp_one_two;
				var vertex;
				//Zero denominator indicates parallel lines
				if (Math.abs(denominator) < 0.0001) {
					vertex = use_one ? point_on_box_one_edge : point_on_box_two_edge;
				} else {
					var mua = (dp_one_two * dp_st_two - sm_two * dp_st_one) / denominator;
					var mub = (sm_one * dp_st_two - dp_one_two * dp_st_one) / denominator;

					//If either of the edges has the nearest point out of bounds, then the edges aren't crossed, we have an edge-face contact.
					var box_one_axis_name = axis_index_to_name(box_one_axis_index);
					var box_two_axis_name = axis_index_to_name(box_two_axis_index);

					if (mua > box_one.halfSize[box_one_axis_name] || mua < -box_one.halfSize[box_one_axis_name] || mub > box_two.halfSize[box_two_axis_name] || mub < -box_two.halfSize[box_two_axis_name]) {
						vertex = use_one ? point_on_box_one_edge : point_on_box_two_edge;
					} else {
						point_on_box_one_edge.add(box_one_axis.scale(mua));
						point_on_box_two_edge.add(box_two_axis.scale(mub));
						vertex = point_on_box_one_edge.scale(0.5).add(point_on_box_two_edge.scale(0.5));
					}
				}

				//Create contact data
				contact = this.__getFreeContact();
				vertex.clone(contact.point);
				axis.clone(contact.normal);
				contact.penetration = penetration;
				contact.setContactData(box_one.body, box_two.body, data.restitution, data.friction);
				contacts.push(contact);
				return true;
			}
			
			return false;
		};

		/**
			Checks for contacts between the two given bodies. This will check for contacts between all the primitives
			in the given bodies.
			@param {Physics.RigidBody} first_body The first body to check.
			@param {Physics.RigidBody} second_body The second body to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.checkForContacts = function(first_body, second_body, data, contacts) {
			var primitive_one, primitive_two, i, j, len, len2, ref2;
			var result = false;
			var ref = first_body.primitives;

			for (i = 0, len = ref.length; i < len; i++) {
				primitive_one = ref[i];
				ref2 = second_body.primitives;
				for (j = 0, len2 = ref2.length; j < len2; j++) {
					//Check each primitive in the first body with the primitives in the second body
					primitive_two = ref2[j];
					if (primitive_one instanceof Primitives.Plane) {
						if (primitive_two instanceof Primitives.Box) {
							result = result || this.boxAndHalfSpace(primitive_two, primitive_one, data, contacts);
						} else if (primitive_two instanceof Primitives.Sphere) {
							result = result || this.sphereAndHalfSpace(primitive_two, primitive_one, data, contacts);
						}
						continue;
					}
						
					if (primitive_one instanceof Primitives.Box) {
						if (primitive_two instanceof Primitives.Plane) {
							result = result || this.boxAndHalfSpace(primitive_one, primitive_two, data, contacts);
						} else if (primitive_two instanceof Primitives.Sphere) {
							result = result || this.boxAndSphere(primitive_one, primitive_two, data, contacts);
						} else if (primitive_two instanceof Primitives.Box) {
							result = result || this.boxAndBox(primitive_one, primitive_two, data, contacts);
						}
						continue;
					}

					if (primitive_one instanceof Primitives.Sphere) {
						if (primitive_two instanceof Primitives.Plane) {
							result = result || this.sphereAndHalfSpace(primitive_one, primitive_two, data, contacts);
						} else if (primitive_two instanceof Primitives.Box) {
							result = result || this.boxAndSphere(primitive_two, primitive_one, data, contacts);
						} else if (primitive_two instanceof Primitives.Sphere) {
							result = result || this.sphereAndSphere(primitive_one, primitive_two, data, contacts);
						}
						continue;
					}
				}
			}
			return result;
		};

		/**
			Checks for contacts between a body and a plane. This will check for contacts between all the primitives
			in the given body.
			@param {Physics.RigidBody} body The body to check.
			@param {Physics.Primitives.Plane} plane The plane to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.checkForContactsWithPlane = function(body, plane, data, contacts) {
			var primitive_one, result, i, len, ref;
			result = false;
			ref = body.primitives;

			for (i = 0, len = ref.length; i < len; i++) {
				primitive_one = ref[i];
				if (primitive_one instanceof Primitives.Box) {
					result = result || this.boxAndHalfSpace(primitive_one, plane, data, contacts);
					continue;
				}
				if (primitive_one instanceof Primitives.Sphere) {
					result = result || this.sphereAndHalfSpace(primitive_one, primitive_two, data, contacts);
					continue;
				}
			}
			return result;
		};
		return CollisionDetector;
	})();
	return CollisionDetector;
});