var sys = require('util');
var fs = require('fs');
var exec = require('child_process').exec;
var requirejs = require('requirejs');

var config = {
	appDir: "src/",
	baseUrl: ".",
	dir: "dist/",
	findNestedDependencies: true,
	modules: [
		{
			name: "goom-physics"
		}
	]
};

desc("This is the default task.");
task("default", function(params) {
	//Do something.
});

desc("Runs all the tests.");
task("test", function(params){
	exec("jasmine-node spec/", function (error, stdout, stderr) {
		sys.print(stdout);
		sys.print(stderr);
	});
});

desc("Builds the project into a minified file.");
task("build", function(params){
	console.log("Building the project into a minified file...")
	requirejs.optimize(config, function (buildResponse) {
		fs.unlink('dist/build.txt');
		fs.unlink('dist/body_force_registry.js');
		fs.unlink('dist/bounding_sphere.js');
		fs.unlink('dist/bounding_volume_hierarchy_node.js');
		fs.unlink('dist/collision_detector.js');
		fs.unlink('dist/contact.js');
		fs.unlink('dist/contact_resolver.js');
		fs.unlink('dist/force_generator.js');
		fs.unlink('dist/gravity.js');
		fs.unlink('dist/intersection_tests.js');
		fs.unlink('dist/primitives.js');
		fs.unlink('dist/world.js');
		fs.unlink('dist/rigid_body.js');
		fs.rename('dist/goom-math.js', 'dist/goom-physics.min.js');
		console.log("The file is ready at dist/goom-physics.min.js");
	});
});