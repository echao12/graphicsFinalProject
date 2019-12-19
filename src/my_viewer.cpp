
# include "my_viewer.h"

# include <sigogl/ui_button.h>
# include <sigogl/ui_radio_button.h>
# include <sig/sn_primitive.h>
# include <sig/sn_transform.h>
# include <sig/sn_manipulator.h>

# include <sigogl/ws_run.h>

MyViewer::MyViewer ( int x, int y, int w, int h, const char* l ) : WsViewer(x,y,w,h,l)
{
	camY = 2;
	camX = camZ = 1.0f;
	cam = false;
	strideNum = 0;
	movingForward = true;
	fixedCamera = true;
	camera().znear = 0.25;
	//initialize the angles counter for each pivot
	for (int index = 0; index < 6; index++) {
		angle[index] = 0;
	}
	_nbut = 0;
	_animating = false;
	build_ui ();
	build_scene ();
}

void MyViewer::build_ui ()
{
	UiPanel *p, *sp;
	UiManager* uim = WsWindow::uim();
	p = uim->add_panel ( "", UiPanel::HorizLeft );
	p->add ( new UiButton ( "View", sp=new UiPanel() ) );
	{	UiPanel* p=sp;
		p->add ( _nbut=new UiCheckButton ( "Normals", EvNormals ) ); 
	}
	p->add ( new UiButton ( "Animate", EvAnimate ) );
	p->add ( new UiButton ( "Exit", EvExit ) ); p->top()->separate();
}

void MyViewer::add_model(SnShape* s, GsVec p) { return;}
void MyViewer::addNPC(GsVec p) {
	// will generate a new NPC at position p
	SnGroup* group[6], * gCharacter, * gBody;// = new SnGroup;
	SnPrimitive* model[6];// , * sModel[6]; // head, body, arms, legs and shadow counter-parts
	SnTransform* trans[6], * tCharacter; // holds transform for each part
	GsMat m[6]; // allows alteration of each transform
	GsBox b[6]; // holds bounding box for each part.
	GsColor c1 = GsColor::random();
	GsColor c2 = GsColor::random();
	GsColor c3 = GsColor::random();
	//steve's head, index 0
	model[0] = new SnPrimitive(GsPrimitive::Box, 0.125f, 0.125f, 0.125f);//radius on each axis
	model[0]->model()->get_bounding_box(b[0]);
	model[0]->prim().material.diffuse = c1;
	trans[0] = new SnTransform;
	trans[0]->set(m[0]);
	group[0] = new SnGroup;
	group[0]->separator(true);
	group[0]->add(trans[0]);
	group[0]->add(model[0]);

	//steve's body
	model[1] = new SnPrimitive(GsPrimitive::Box, 0.125f, 0.25f, 0.07f);
	model[1]->model()->get_bounding_box(b[1]);
	model[1]->prim().material.diffuse = c2;
	trans[1] = new SnTransform;
	m[1].translation(GsVec(0.0f, (-1 * b[0].dy() / 2.0f) + (-1 * b[1].dy() / 2.0f), 0.0f));
	trans[1]->set(m[1]);
	group[1] = new SnGroup;
	group[1]->separator(true);
	group[1]->add(trans[1]);
	group[1]->add(model[1]);

	//left arm
	model[3] = new SnPrimitive(GsPrimitive::Box, 0.125f / 2, 0.25f, 0.07f);
	model[3]->model()->get_bounding_box(b[3]);
	model[3]->prim().material.diffuse = c1;
	trans[3] = new SnTransform;
	m[3].translation(GsVec((-1 * b[1].dx() / 2) + (-1 * b[3].dx() / 2), (-1 * b[0].dy() / 2.0f) + (-1 * b[1].dy() / 2.0f), 0.0f));
	trans[3]->set(m[3]);
	group[3] = new SnGroup;
	group[3]->separator(true);
	group[3]->add(trans[3]);
	group[3]->add(model[3]);

	//right arm
	model[2] = new SnPrimitive(GsPrimitive::Box, 0.125f / 2, 0.25f, 0.07f);
	model[2]->model()->get_bounding_box(b[2]);
	model[2]->prim().material.diffuse = c1;
	trans[2] = new SnTransform;
	m[2].translation(GsVec((b[1].dx() / 2) + (b[2].dx() / 2), (-1 * b[0].dy() / 2.0f) + (-1 * b[1].dy() / 2.0f), 0.0f));
	trans[2]->set(m[2]);
	group[2] = new SnGroup;
	group[2]->separator(true);
	group[2]->add(trans[2]);
	group[2]->add(model[2]);

	//left leg
	model[5] = new SnPrimitive(GsPrimitive::Box, b[1].dx() / 4, b[1].dy() / 2, b[1].dz() / 2);
	model[5]->model()->get_bounding_box(b[5]);
	model[5]->prim().material.diffuse = c3;
	trans[5] = new SnTransform;
	m[5].translation(GsVec((-1 * b[1].dx() / 4), (-1 * b[1].dy()) + (-1 * b[0].dy() / 2) + (-1 * b[5].dy() / 2), 0.0f));
	trans[5]->set(m[5]);
	group[5] = new SnGroup;
	group[5]->separator(true);
	group[5]->add(trans[5]);
	group[5]->add(model[5]);

	//right leg
	model[4] = new SnPrimitive(GsPrimitive::Box, b[1].dx() / 4, b[1].dy() / 2, b[1].dz() / 2);
	model[4]->model()->get_bounding_box(b[4]);
	model[4]->prim().material.diffuse = c3;
	trans[4] = new SnTransform;
	m[4].translation(GsVec((b[1].dx() / 4), (-1 * b[1].dy()) + (-1 * b[0].dy() / 2) + (-1 * b[4].dy() / 2), 0.0f));
	trans[4]->set(m[4]);
	group[4] = new SnGroup;
	group[4]->separator(true);
	group[4]->add(trans[4]);
	group[4]->add(model[4]);

	GsMat cMat;
	cMat.scaling(10.0f);
	cMat.setrans(GsVec(p.x, 10.0f + ((b[0].dy() / 2) + (b[1].dy()) + (b[4].dy())) + p.y, p.z));
	tCharacter = new SnTransform;
	tCharacter->set(cMat);
	//tCharacter->get().translation(GsVec(0.0f, (b[0].dy() / 2) + (b[1].dy()) + (b[4].dy()), 0.0f));
	gCharacter = new SnGroup;
	gBody = new SnGroup;

	gBody->separator(true);
	for (int i = 0; i < 6; i++) {
		gBody->add(group[i]);//attach each part into a character group
	}
	gCharacter->separator(true);
	gCharacter->add(tCharacter);
	gCharacter->add(gBody);
	rootg()->add(gCharacter);

}

void MyViewer::buildCharacter() {

	SnGroup *group[6], *gCharacter, *gBody;// = new SnGroup;
	SnPrimitive* model[6];// , * sModel[6]; // head, body, arms, legs and shadow counter-parts
	SnTransform* trans[6], *tCharacter; // holds transform for each part
	GsMat m[6]; // allows alteration of each transform
	GsBox b[6]; // holds bounding box for each part.
	//steve's head, index 0
	model[0] = new SnPrimitive(GsPrimitive::Box, 0.125f, 0.125f, 0.125f);//radius on each axis
	model[0]->model()->get_bounding_box(b[0]);
	model[0]->prim().material.diffuse = GsColor::brown;
	trans[0] = new SnTransform;
	trans[0]->set(m[0]);
	group[0] = new SnGroup;
	group[0]->separator(true);
	group[0]->add(trans[0]);
	group[0]->add(model[0]);

	//steve's body
	model[1] = new SnPrimitive(GsPrimitive::Box, 0.125f, 0.25f, 0.07f);
	model[1]->model()->get_bounding_box(b[1]);
	model[1]->prim().material.diffuse = GsColor::cyan;
	trans[1] = new SnTransform;
	m[1].translation(GsVec(0.0f, (-1 * b[0].dy() / 2.0f) + (-1 * b[1].dy() / 2.0f), 0.0f));
	trans[1]->set(m[1]);
	group[1] = new SnGroup;
	group[1]->separator(true);
	group[1]->add(trans[1]);
	group[1]->add(model[1]);

	//left arm
	model[3] = new SnPrimitive(GsPrimitive::Box, 0.125f / 2, 0.25f, 0.07f);
	model[3]->model()->get_bounding_box(b[3]);
	model[3]->prim().material.diffuse = GsColor::brown;
	trans[3] = new SnTransform;
	m[3].translation(GsVec((-1 * b[1].dx() / 2) + (-1 * b[3].dx() / 2), (-1 * b[0].dy() / 2.0f) + (-1 * b[1].dy() / 2.0f), 0.0f));
	trans[3]->set(m[3]);
	group[3] = new SnGroup;
	group[3]->separator(true);
	group[3]->add(trans[3]);
	group[3]->add(model[3]);

	//right arm
	model[2] = new SnPrimitive(GsPrimitive::Box, 0.125f / 2, 0.25f, 0.07f);
	model[2]->model()->get_bounding_box(b[2]);
	model[2]->prim().material.diffuse = GsColor::brown;
	trans[2] = new SnTransform;
	m[2].translation(GsVec((b[1].dx() / 2) + (b[2].dx() / 2), (-1 * b[0].dy() / 2.0f) + (-1 * b[1].dy() / 2.0f), 0.0f));
	trans[2]->set(m[2]);
	group[2] = new SnGroup;
	group[2]->separator(true);
	group[2]->add(trans[2]);
	group[2]->add(model[2]);

	//left leg
	model[5] = new SnPrimitive(GsPrimitive::Box, b[1].dx() / 4, b[1].dy() / 2, b[1].dz() / 2);
	model[5]->model()->get_bounding_box(b[5]);
	model[5]->prim().material.diffuse = GsColor::blue;
	trans[5] = new SnTransform;
	m[5].translation(GsVec((-1 * b[1].dx() / 4), (-1 * b[1].dy()) + (-1 * b[0].dy() / 2) + (-1 * b[5].dy() / 2), 0.0f));
	trans[5]->set(m[5]);
	group[5] = new SnGroup;
	group[5]->separator(true);
	group[5]->add(trans[5]);
	group[5]->add(model[5]);

	//right leg
	model[4] = new SnPrimitive(GsPrimitive::Box, b[1].dx() / 4, b[1].dy() / 2, b[1].dz() / 2);
	model[4]->model()->get_bounding_box(b[4]);
	model[4]->prim().material.diffuse = GsColor::blue;
	trans[4] = new SnTransform;
	m[4].translation(GsVec((b[1].dx() / 4), (-1 * b[1].dy()) + (-1 * b[0].dy() / 2) + (-1 * b[4].dy() / 2), 0.0f));
	trans[4]->set(m[4]);
	group[4] = new SnGroup;
	group[4]->separator(true);
	group[4]->add(trans[4]);
	group[4]->add(model[4]);

	GsMat cMat;
	cMat.scaling(10.0f);
	cMat.setrans(GsVec(0.0f, 10.0f + ((b[0].dy() / 2) + (b[1].dy()) + (b[4].dy())), 0.0f));
	tCharacter = new SnTransform;
	tCharacter->set(cMat);
	//tCharacter->get().translation(GsVec(0.0f, (b[0].dy() / 2) + (b[1].dy()) + (b[4].dy()), 0.0f));
	gCharacter = new SnGroup;
	gBody = new SnGroup;

	gBody->separator(true);
	for (int i = 0; i < 6; i++) {
		gBody->add(group[i]);//attach each part into a character group
	}
	gCharacter->separator(true);
	gCharacter->add(tCharacter);
	gCharacter->add(gBody);
	rootg()->add(gCharacter);
}
void MyViewer::buildEnvironment() {
	SnGroup* eGroup, * floorGroup, * towerGroup;//floor group will be attatched to egroup.
	SnModel* floor;//the city model
	SnModel* tower;//the tower model
	GsBox fBox;
	GsMat fMat;


	/*CITY*/
	//floor = new SnPrimitive(GsPrimitive::Box, 5.0f, 0.25f, 5.0f);
	//floor->prim().material.diffuse = GsColor::green;
	floor = new SnModel;
	floor->model()->load("../EnvironmentModels/city.obj");
	floor->model()->get_bounding_box(fBox);
	//floor->prim().get_bounding_box(fBox);

	SnTransform* floorTrans, * globalTrans;
	globalTrans = new SnTransform;//will gobablly move all objects attatched to egroup
	floorTrans = new SnTransform;//will move the city model independently

	//fMat.translation(0, fBox.dy()/-2.0f, 0);
	//fMat.translation(0, fBox.dy()*-1, 0);

	fMat.scaling(0.25);//SHRINKING DOWN THE CITY MODEL. 0.25 IS THE LOWER THRESHOLD BEFORE ERRORS ARE THROWN
	fMat.setrans(GsVec((fBox.dx() / 4) / 5.0f, 29.0f, 0.0f));// THIS GLOBALLY MOVES THE CITY MODEL. THE TRANSLATION IS SEPARATE FROM EVERYTHING ELSE
	floorTrans->set(fMat);

	floorGroup = new SnGroup;
	floorGroup->separator(true);
	floorGroup->add(floorTrans);
	floorGroup->add(floor);

	eGroup = new SnGroup;
	eGroup->separator(true);
	eGroup->add(globalTrans);//egroup->0
	eGroup->add(floorGroup);//egroup->1


	/*CAR*/
	SnGroup* houseG;
	SnModel* houseM;
	SnTransform* houseT;
	GsMat houseMat;
	GsBox houseB;

	houseM = new SnModel;
	houseM->model()->load("../Car-Model/Car.obj");
	houseM->model()->get_bounding_box(houseB);
	houseMat.scaling(5.0f);
	houseMat.setrans(GsVec(0.0f, 0.0f, -3.0f));
	GsMat rotation;
	rotation.roty(float(GS_PI));
	houseMat.mult(rotation, houseMat);
	houseT = new SnTransform;
	houseT->set(houseMat);

	houseG = new SnGroup;
	houseG->separator(true);
	houseG->add(houseT);
	houseG->add(houseM);

	eGroup->add(houseG);//eGroup->2

	/*TOWER*/
	tower = new SnModel;
	GsBox tBox;
	GsMat tMat;
	if (!tower->model()->load("../EnvironmentModels/tower.obj")) {
		gsout << "tower not loaded";
	}
	tower->model()->get_bounding_box(tBox);


	SnTransform* towerTrans, * gTrans;
	gTrans = new SnTransform;//will gobablly move all objects attatched to egroup
	towerTrans = new SnTransform;//will move the tower model independently

	//fMat.translation(0, fBox.dy()/-2.0f, 0);
	//fMat.translation(0, fBox.dy()*-1, 0);

	tMat.scaling(30.0f);
	tMat.setrans(GsVec((fBox.dx() / 4.5f) / 5.5f, -(fBox.dy() / 6.5f) / 2.0f, (fBox.dz() / 4) / 5.0f));// THIS GLOBALLY MOVES THE TOWER MODEL. THE TRANSLATION IS SEPARATE FROM EVERYTHING ELSE
	towerTrans->set(tMat);

	towerGroup = new SnGroup;
	towerGroup->separator(true);
	towerGroup->add(towerTrans);
	towerGroup->add(tower);

	//eGroup = new SnGroup;
	//eGroup->separator(true);
	//eGroup->add(gTrans);//egroup->3
	eGroup->add(towerGroup);//egroup->4


	rootg()->add(eGroup);

}
int factorial(int i) {
	if (i < 2)
		return 1;
	else
		return i * factorial(i - 1);
}
GsPnt MyViewer::eval_Bezier(float t, const GsArray<GsPnt>& P) {
	//evaluating the point on the curve given the control points
	GsPnt point;//the point we are evauating
	int n = P.size() - 1;
	for (int i = 0; i < n + 1; i++) {
		point += (P[i] * ((factorial(n) / (factorial(i) * factorial(n - i)))
			* powf(t, (float)i) * powf(1.0f - t, (float)(n - i))));
	}
	return point;
}
void MyViewer::generatePaths() {
	SnLines *path = new SnLines;//create magenta path
	SnLines *paths[2];//create another path as array to make it easier to add more later
	GsArray<GsPnt> controlPnts;//generate control points for magenta curve
	GsArray<GsPnt> cPnts[2];
	paths[0] = new SnLines;

	path->init();
	path->color(GsColor::magenta);
	path->line_width(3.5f);

	paths[0]->init();
	paths[0]->color(GsColor::cyan);
	paths[0]->line_width(3.5f);

	//GsArray<GsPnt>* curvePnts;//will need to keep curvePnts
	SnGroup* pathG = new SnGroup;//will be a subgroup of environment
	pathG->separator(true);

	float height = 0.25f;
	//for the first curve. the magenta one
	controlPnts.push() = GsPnt(-10, height, 385);
	controlPnts.push() = GsPnt(-10, height, 0);
	controlPnts.push() = GsPnt(-10, height, -165);

	cPnts[0].push() = GsPnt(10, height, 385);
	cPnts[0].push() = GsPnt(10, height, 0);
	cPnts[0].push() = GsPnt(10, height, -165);

	//generate path for robot to walk

	paths[1] = new SnLines;
	paths[1]->init();
	paths[1]->color(GsColor::yellow);
	paths[1]->line_width(1.5f);

	cPnts[1].push() = GsPnt(50, height*100, -50);
	cPnts[1].push() = GsPnt(50, height*100, 50);


	//controlPnts.push() = GsPnt(-2, height, 20);
	float delta = 0.125f/8.0f;

	path->begin_polyline();
	paths[0]->begin_polyline();
	paths[1]->begin_polyline();
	for (float t = 0; t < 1.0f; t += delta) {
		//for the first curve. magenta one
		GsPnt pnt = eval_Bezier(t, controlPnts);
		path->push(pnt);//to display the curve
		//for the 2nd curve, cyan
		pnt = eval_Bezier(t, cPnts[0]);
		paths[0]->push(pnt);

		pnt = eval_Bezier(t, cPnts[1]);
		paths[1]->push(pnt);
	}
	path->end_polyline();
	paths[0]->end_polyline();
	paths[1]->end_polyline();

	pathG->add(path);//0
	pathG->add(paths[0]);//1
	pathG->add(paths[1]);
	rootg()->add(pathG);
}
void MyViewer::buildCars() {
	SnGroup *carPart[6], *gCar, *gBody;// = new SnGroup;
	SnPrimitive *part[6];// , * sModel[6]; // head, body, arms, legs and shadow counter-parts
	SnTransform *trans[6], *tCar; // holds transform for each part
	GsMat m[6]; // allows alteration of each transform
	GsMat rotation;
	rotation.rotz(gspidiv2);
	GsBox b[6]; // holds bounding box for each part.

	for (int i = 0; i < 6; i++) {
		carPart[i] = new SnGroup;
		carPart[i]->separator(true);
		trans[i] = new SnTransform;
	}
	gCar = new SnGroup;
	gCar->separator(true);
	tCar = new SnTransform;
	gBody = new SnGroup;
	gBody->separator(true);

	//car parts: 0top 1body 2FrontLeft 3FrontRight 4BackLeft 5BackRight
	//main body
	part[0] = new SnPrimitive(GsPrimitive::Box, 0.5f, 0.2f, 0.75f);
	part[0]->model()->get_bounding_box(b[0]);
	part[0]->prim().material.diffuse = GsColor::red;
	carPart[0]->add(trans[0]);
	carPart[0]->add(part[0]);
	//top body
	part[1] = new SnPrimitive(GsPrimitive::Box, 0.5f, 0.15f, 0.25f);
	part[1]->prim().material.diffuse = GsColor::red;
	part[1]->model()->get_bounding_box(b[1]);
	carPart[1]->add(trans[1]);
	m[1].setrans(GsVec(0.0f,(b[0].dy()/2) + b[1].dy()/2,0.0f));
	trans[1]->set(m[1]);
	carPart[1]->add(part[1]);

	//Front Left wheel
	part[2] = new SnPrimitive(GsPrimitive::Cylinder, 0.25f, 0.25f, 0.05f);
	part[2]->prim().material.diffuse = GsColor::black;
	part[2]->get_bounding_box(b[2]);
	carPart[2]->add(trans[2]);
	m[2].mult(rotation, m[2]);
	m[2].setrans(GsVec((b[0].dx() / 2) + (b[2].dx()/8), b[0].dy() / -3, (b[0].dz() / 2)-(b[2].dz()/2)));

	trans[2]->set(m[2]);
	carPart[2]->add(part[2]);

	//Front Right Wheel
	part[3] = new SnPrimitive(GsPrimitive::Cylinder, 0.25f, 0.25f, 0.05f);
	part[3]->prim().material.diffuse = GsColor::black;
	part[3]->get_bounding_box(b[3]);
	carPart[3]->add(trans[3]);
	m[3].mult(rotation, m[3]);
	m[3].setrans(GsVec((b[0].dx() / -2) + (b[3].dx() / -8), b[0].dy() / -3, (b[0].dz() / 2) - (b[3].dz() / 2)));

	trans[3]->set(m[3]);
	carPart[3]->add(part[3]);

	//Back Left wheel
	part[4] = new SnPrimitive(GsPrimitive::Cylinder, 0.25f, 0.25f, 0.05f);
	part[4]->prim().material.diffuse = GsColor::black;
	part[4]->get_bounding_box(b[4]);
	carPart[4]->add(trans[4]);
	m[4].mult(rotation, m[4]);
	m[4].setrans(GsVec((b[0].dx() / 2) + (b[4].dx() / 8), b[0].dy() / -3, (b[0].dz() / -2) - (b[4].dz() / -2)));

	trans[4]->set(m[4]);
	carPart[4]->add(part[4]);

	//Back Right wheel
	part[5] = new SnPrimitive(GsPrimitive::Cylinder, 0.25f, 0.25f, 0.05f);
	part[5]->prim().material.diffuse = GsColor::black;
	part[5]->get_bounding_box(b[5]);
	carPart[5]->add(trans[5]);
	m[5].mult(rotation, m[5]);
	m[5].setrans(GsVec((b[0].dx() / -2) + (b[5].dx() / -8), b[0].dy() / -3, (b[0].dz() / -2) - (b[5].dz() / -2)));

	trans[5]->set(m[5]);
	carPart[5]->add(part[5]);

	//add global transform to control the body of the car
	gBody->add(tCar);//order: root->gCar->gBody(tCar, carPart[i](carTrans,part Model))
	GsMat raiseCar;
	raiseCar.translation(GsVec(0, (b[0].dy()),0));
	tCar->set(raiseCar);
	//add carParts to the body
	for (int i = 0; i < 6; i++) {
		gBody->add(carPart[i]);
	}
	//might want to place tCar outside of the gBody node.
	gCar->add(gBody);
	rootg()->add(gCar);

}
void MyViewer::buildRobot() {

	SnGroup *mg = new SnGroup;
	SnTransform *mt = new SnTransform;
	GsMat m[11];
	SnTransform* t[6];
	SnGroup* g[6];
	SnModel* model[11];

	//.................................The following are the parts.

	t[0] = new SnTransform;
	t[0]->set(m[0]);
	g[0] = new SnGroup;
	//g[0]->add(t[0]);//adding translation of the first matrix to the root.


	//body 
	model[0] = new SnModel;
	model[0]->model()->make_cylinder(GsPnt(0.0f, 3.0f, 0.0f), GsPnt(0.0f, -3.0f, 0.0f), 2.0f, 2.0f, 20, true);
	model[0]->color(GsColor::red);
	g[0]->add(t[0]);
	g[0]->add(model[0]);
	g[0]->separator(true);



	//....................The neck of character

	t[1] = new SnTransform;
	t[1]->set(m[1]);

	model[1] = new SnModel;
	g[1] = new SnGroup;
	model[1]->model()->make_cylinder(GsPnt(0.0f, 4.5f, 0.0f), GsPnt(0.0f, 3.0f, 0.0f), 0.2f, 0.2f, 20, true);
	model[1]->color(GsColor::blue);
	g[1]->add(t[1]);
	g[1]->add(model[1]);

	//..................................The head of character
	model[2] = new SnModel;
	model[2]->model()->make_cylinder(GsPnt(0.0f, 5.7f, 0.0f), GsPnt(0.0f, 4.5f, 0.0f), 0.9f, 0.9f, 20, true);
	model[2]->color(GsColor::red);
	g[1]->add(model[2]);
	g[1]->separator(true);



	//	.//.........///........//...............right arm
	t[2] = new SnTransform;
	g[2] = new SnGroup;
	m[2].translation(-2.0f, 0.0f, 0.0f);
	t[2]->set(m[2]);

	model[3] = new SnModel;
	model[3]->model()->make_cylinder(GsPnt(-5.0f, 1.0f, 0.0f), GsPnt(0.0f, 1.0f, 0.0f), 0.75f, 0.75f, 20, true);
	model[3]->color(GsColor::blue);
	g[2]->add(t[2]);
	g[2]->add(model[3]);


	//.....................................R
	model[4] = new SnModel;
	model[4]->model()->make_cylinder(GsPnt(-7.0f, 1.0f, 0.0f), GsPnt(-5.0f, 1.0f, 0.0f), 0.5f, 0.5f, 20, true);
	model[4]->color(GsColor::red);
	g[2]->add(model[4]);
	g[2]->separator(true);



	//....///.........///.........//............left arm
	t[3] = new SnTransform;
	g[3] = new SnGroup;
	m[3].translation(2.0f, 0.0f, 0.0f);
	t[3]->set(m[3]);

	model[5] = new SnModel;
	model[5]->model()->make_cylinder(GsPnt(0.0f, 1.0f, 0.0f), GsPnt(5.0f, 1.0f, 0.0f), 0.75f, 0.75f, 20, true);
	model[5]->color(GsColor::blue);
	g[3]->add(t[3]);
	g[3]->add(model[5]);

	//..................................L..
	model[6] = new SnModel;
	model[6]->model()->make_cylinder(GsPnt(5.0f, 1.0f, 0.0f), GsPnt(7.0f, 1.0f, 0.0f), 0.5f, 0.5f, 20, true);
	model[6]->color(GsColor::red);
	g[3]->add(model[6]);
	g[3]->separator(true);


	//.....//.......//.......//...............Right leg

	t[4] = new SnTransform;
	g[4] = new SnGroup;
	/*m[4].translation(0.0f, -2.0f, 0.0f);
	t[4]->set(m[4]);*/

	model[7] = new SnModel;
	model[7]->model()->make_cylinder(GsPnt(1.0f, -3.0f, 0.0f), GsPnt(1.0f, -6.0f, 0.0f), 0.5f, 0.5f, 20, true);
	model[7]->color(GsColor::blue);
	g[4]->add(t[4]);
	g[4]->add(model[7]);
	//..................................Rl...
	model[8] = new SnModel;
	model[8]->model()->make_cylinder(GsPnt(1.0f, -6.0f, 0.0f), GsPnt(1.0f, -8.0f, 0.0f), 0.5f, 0.5f, 20, true);
	model[8]->color(GsColor::red);
	g[4]->add(model[8]);
	g[4]->separator(true);




	//..........//........//.........//......leftleg
	t[5] = new SnTransform;
	g[5] = new SnGroup;
	/*m[5].translation(0.0f, -2.0f, 0.0f);
	t[5]->set(m[5]);*/

	model[9] = new SnModel;
	//g[5] = new SnGroup;
	model[9]->model()->make_cylinder(GsPnt(-1.0f, -3.0f, 0.0f), GsPnt(-1.0f, -6.0f, 0.0f), 0.5f, 0.5f, 20, true);
	model[9]->color(GsColor::blue);
	g[5]->add(t[5]);
	g[5]->add(model[9]);

	//..................................l...
	model[10] = new SnModel;
	model[10]->model()->make_cylinder(GsPnt(-1.0f, -6.0f, 0.0f), GsPnt(-1.0f, -8.0f, 0.0f), 0.5f, 0.5f, 20, true);
	model[10]->color(GsColor::red);

	g[5]->add(model[10]);
	g[5]->separator(true);
	//......//.......//......//......................


	GsMat matrix;
	SnGroup* Char = new SnGroup;
	Char->separator(true);
	Char->add(mt);//controls the global transform for the robot body
	matrix.scaling(2.0f);
	matrix.setrans(GsVec(0.0f, 18.0f, -5.0f));
	mt->set(matrix);
	for (int i = 0; i < 6; i++)
	{
		mg->add(g[i]);
	}
	Char->add(mg);
	rootg()->add(Char);
}
void MyViewer::animateRobot() {
	//will generate robot animation.
	GsMat rotation;
	GsMat tp, tn;
	GsVec pos;

	//if (_animating) return; // avoid recursive calls
	//_animating = true;
	//*note* t0->head&neck t2-> right arm t3-> left arm t4->right leg t5-> left leg
	SnTransform* t[6];
	//we only neeed t5 and t4;
	t[5] = (rootg()->get<SnGroup>(3)->get<SnGroup>(1))->get<SnGroup>(5)->get<SnTransform>(0);
	t[4] = (rootg()->get<SnGroup>(3)->get<SnGroup>(1))->get<SnGroup>(4)->get<SnTransform>(0);
	
	// ANIMATION PART FOR MOVING LEGS 
	double frdt = 1.0 / 30.0; // delta time to reach given number of frames per second
	static float rightlegangle = 0, leftlegangle = 0;
	static double ti = 0, lt = 0, t0 = gs_time();//t0 current
	//do // run for a while:
	//{
		while (ti - lt < frdt) { ws_check(); ti = gs_time() - t0; } // wait until it is time for next frame


		if (ti >= 0 && ti < 0.4)
		{
			if (rightlegangle + ((float)GS_PIDIV2 / 20.0f) < (float)GS_PIDIV2 / 3) {
				rightlegangle += (float)GS_PIDIV2 / 20.0f;
				rotation.rotx((float)GS_PIDIV2 / 20.0f);
				t[4]->get().mult(t[4]->get(), rotation);
			}
		}

		else if (ti >= 0.4 && ti < 0.8)

		{
			if (rightlegangle - ((float)GS_PIDIV2 / 20.0f) >= 0) {
				rightlegangle = rightlegangle - (float)GS_PIDIV2 / 20.0f;
				rotation.rotx(-1 * (float)GS_PIDIV2 / 20.0f);
				t[4]->get().mult(t[4]->get(), rotation);
			}

		}
		else if (ti >= 0.8 && ti < 1.2)

		{
			if (leftlegangle + ((float)GS_PIDIV2 / 20.0f) < (float)GS_PIDIV2 / 3) {
				leftlegangle = leftlegangle + (float)GS_PIDIV2 / 20.0f;
				rotation.rotx((float)GS_PIDIV2 / 20.0f);
				t[5]->get().mult(t[5]->get(), rotation);
			}

		}

		else if (ti >= 1.2 && ti < 1.6)

		{
			if (leftlegangle + ((float)GS_PIDIV2 / 20.0f) >= 0) {
				leftlegangle = leftlegangle - (float)GS_PIDIV2 / 20.0f;
				rotation.rotx(-1 * (float)GS_PIDIV2 / 20.0f);
				t[5]->get().mult(t[5]->get(), rotation);
			}

		}
		//gsout << ti << gsnl;
		if (ti >= 1.6)
		{
			ti = 0, lt = 0, t0 = gs_time();
		}
		else {

			lt = ti;
		}


		render(); // notify it needs redraw
		ws_check(); // redraw now
	//} while (1); //put some condition to run -suggestion
	//_animating = false;

}

void MyViewer::spawnNPC() {
	// these won't be moving. to make it easier to control
	// specific npc's, spawn those first above this line
	addNPC(GsVec(90, 0, 90));
	addNPC(GsVec(50, 0, 65));
	addNPC(GsVec(58, 0, 60));
	addNPC(GsVec(53, 0, 53));
	addNPC(GsVec(-50, 0, 95));
	addNPC(GsVec(-58, 0, 80));
	addNPC(GsVec(-90, 0, 90));

	addNPC(GsVec(-90, 0, 160));
	addNPC(GsVec(53, 0, 160));
	addNPC(GsVec(50, 0, 163));
	addNPC(GsVec(-50,0,170));

	addNPC(GsVec(-190, 0, 160));
	addNPC(GsVec(-187, 0, 170));
	addNPC(GsVec(-190, 0, 270));
	addNPC(GsVec(-189, 0, 290));

	addNPC(GsVec(-45, 0, 195));
	addNPC(GsVec(-55, 0, 180));

	addNPC(GsVec(150, 0, 65));
	addNPC(GsVec(158, 0, 60));
	addNPC(GsVec(153, 0, 70));

	addNPC(GsVec(202, 0, 65));
	addNPC(GsVec(208, 0, 60));
	addNPC(GsVec(203, 0, 70));
	addNPC(GsVec(208, 0, 60));
	addNPC(GsVec(205, 0, 55));
	addNPC(GsVec(203, 0, 58));
}

void MyViewer::build_scene ()
{
	// this project will have 3 main groups from root
	//0->environment 1->characters
	//environment sub groups: 
	buildEnvironment();//attatch egroup(separator true) to root. egroup consists of global transform, floor group(sep. true), house group(sep true).
	buildCharacter();//steve is index 1 from root. adds character group to root(sep. true). this group consists of global transform and body group(sep true)
	generatePaths();//rootg->2
	buildRobot();// rootg->3
	buildCars();// rootg ->4
	spawnNPC();
}
void moveNPC() {
	//will move the npc's along the dedicated path
}
void MyViewer::moveCars() {
	if (CarMoving == false)
		return;
	//get car model
	SnTransform* car1T = ((rootg()->get<SnGroup>(0))->get<SnGroup>(2))->get<SnTransform>(0);
	GsMat mat;
	//static so we can track this variable through this function
	static int index = 0;//controls what index of the curve we are on
	//get curve array
	GsArray<GsPnt>* curvePnts = &((rootg()->get<SnGroup>(2))->get<SnLines>(0)->V);
	//check if index goes out of bounds
	if (index >= curvePnts->size())
		index = 0;//reset the variable
	mat.set(car1T->get());//get old transform matrix
	mat.setrans(GsVec(curvePnts->get(index).x, curvePnts->get(index).y, curvePnts->get(index).z));//set new translation coordinate w/o affecting scaling matrix
	car1T->set(mat);
	index++;
	//gsout << "index: " << index << gsnl;
	return;
}
void MyViewer::moveRobot() {
	//will move global transform of robot along a curve.
	//will use path 2(yellow curve)
	GsArray<GsPnt>* pathPnts = &((rootg()->get<SnGroup>(2)->get<SnLines>(2))->V);
	//get transform of the robot
	SnTransform* robT = rootg()->get<SnGroup>(3)->get<SnTransform>(0);
	static int index = 0;
	GsMat trans;
	if (index >= pathPnts->size()) {
		index = 0;
	}
	trans.set(robT->get());
	trans.setrans(GsVec(pathPnts->get(index).x, pathPnts->get(index).y, pathPnts->get(index).z));
	robT->set(trans);
	index++;
	return;
}
// Below is an example of how to control the main loop of an animation:
void MyViewer::run_animation ()
{
	
	if ( _animating ) return; // avoid recursive calls
	_animating = true;
	CarMoving = false;
	int index = 0;
	static int robI = 0;
	double frdt = 1.0/30.0; // delta time to reach given number of frames per second
	//double v = 4; // target velocity is 1 unit per second
	double t=0, lt=0, t0=gs_time(), x=0, x0 = gs_time();
	static float addz = 15.0f, addy = 15.0f;
	//int ind = gs_random(0, rootg()->size() - 1); // pick one child
	//SnManipulator* manip = rootg()->get<SnManipulator>(ind); // access one of the manipulators
	//GsMat m = manip->mat();
	
	do {// run for a while:
		while ( t-lt<frdt ) { 
			ws_check(); 
			t=gs_time()-t0; 
	} // wait until it is time for n
		//ext frame
		//double yinc = (t-lt)*v;
		x = gs_time() - x0;
		if (x > 0.025f) { // after x secs
			CarMoving = true;
			moveCars();
			animateRobot();
			if(robI%3 == 0)
				moveRobot();
			CarMoving = false;
			robI++;
			x0 = gs_time();//update t0 so that it resets t to 0
		}
		//lt = t;//update lastTime
		
		lt = gs_time() - t0;
		// -= would zoom in
		// += would zoom out


		if (cam == false) {
			if (t > 0 && t < 2) {
				camera().center.y = 100.0f;
				camera().fovy -= 0.01f;
			}
			else if (t > 2 && t < 4) {
				camera().eye.x -= 1.0f;
				camera().center.y = 100.0f;
			}
			else if (t > 4 && t < 6) {
				camera().eye.x -= 1.0f;
				camera().center.y = 100.0f;
			}
			else if (t > 6 && t < 9) {
				camera().eye.x -= 5.0f;
				camera().center.y = 100.0f;
			}
			else if (t > 9 && t < 21) {
				camera().eye.z -= 5.0f;
				camera().center.y = 100.0f;
			}
			else if (t > 21 && t < 27) {
				camera().center.y = 100.0f;
				camera().eye.x += 5.0f;
			}
			else if (t > 27 && t < 38) {
				camera().center.y = 100.0f;
				camera().eye.z += 5.0f;
			}
			else if (t > 38 && t < 40) {
				camera().center.y += 2.0f;
				camera().eye.x -= 5.0f;
			}
			else if (t > 40 && t < 44) {
				camera().center.x += 0.5f;
			}
			else if (t > 44 && t < 45.7) {
				camera().center.y = 200.0f;
				camera().center.x = 100.0f;
				camera().center.z = 150.0f;
				camera().fovy -= 0.02f;
			}
			else if (t > 45.7 && t < 48) {
				camera().center.y = 200.0f;
				camera().center.x = 100.0f;
				camera().center.z = 150.0f;
				camera().eye.z = -650.0f;
				camera().fovy += 0.02f;

			}
		}

		// This is the parameter for the pause camera 
		else {
			/*camera().eye.z = -648.046f;
			camera().eye.y = 209.23f;
			camera().eye.x = 7.60592f;
			camera().center.z = 0.237752f;
			camera().center.y = 15.6944f;
			camera().center.x = 22.9082f;*/

			SnTransform* headT = rootg()->get<SnGroup>(1)->get<SnTransform>(0);
			static bool jumped = false;
			if (jumped == false) {
				//jump to character
				camera().eye.z = headT->get().e34 + addz;
				camera().eye.y = headT->get().e24 + addy;
				camera().eye.x = headT->get().e14;
				camera().center.z = headT->get().e34;
				camera().center.y = headT->get().e24;
				camera().center.x = headT->get().e14;
				render();
				ws_check();
				jumped = true;
			}
			while (addz < 75.0f) {
				render();
				ws_check();
				addz += 0.25f;
				addy += 0.25f;
				camera().eye.z = headT->get().e34 + addz;
				camera().eye.y = headT->get().e24 + addy;
				camera().eye.y = headT->get().e24 + addy;
				render();
				ws_check();
			}
				camera().eye.z = headT->get().e34 + addz;
				camera().eye.y = headT->get().e24 + addy;
				camera().eye.x = headT->get().e14;
				camera().center.z = headT->get().e34;
				camera().center.y = headT->get().e24;
				camera().center.x = headT->get().e14;
				render();
				ws_check();
		}

		render(); // notify it needs redraw
		ws_check(); // redraw now
		message().setf("local time=%f", lt);
	}	while ( 1 );
	_animating = false;
}

void MyViewer::show_normals ( bool view )
{
	// Note that primitives are only converted to meshes in GsModel
	// at the first draw call.
	GsArray<GsVec> fn;
	SnGroup* r = (SnGroup*)root();
	for ( int k=0; k<r->size(); k++ )
	{	SnManipulator* manip = r->get<SnManipulator>(k);
		SnShape* s = manip->child<SnGroup>()->get<SnShape>(0);
		SnLines* l = manip->child<SnGroup>()->get<SnLines>(1);
		if ( !view ) { l->visible(false); continue; }
		l->visible ( true );
		if ( !l->empty() ) continue; // build only once
		l->init();
		if ( s->instance_name()==SnPrimitive::class_name )
		{	GsModel& m = *((SnModel*)s)->model();
			m.get_normals_per_face ( fn );
			const GsVec* n = fn.pt();
			float f = 0.33f;
			for ( int i=0; i<m.F.size(); i++ )
			{	const GsVec& a=m.V[m.F[i].a]; l->push ( a, a+(*n++)*f );
				const GsVec& b=m.V[m.F[i].b]; l->push ( b, b+(*n++)*f );
				const GsVec& c=m.V[m.F[i].c]; l->push ( c, c+(*n++)*f );
			}
		}  
	}
}

int MyViewer::handle_keyboard ( const GsEvent &e )
{
	int ret = WsViewer::handle_keyboard(e); // 1st let system check events
	if (ret) return ret;
	//get character group
	SnGroup* gCharacter = (rootg()->get<SnGroup>(1))->get<SnGroup>(1);
	SnGroup* gHead = gCharacter->get<SnGroup>(0);
	SnGroup* gLHand = gCharacter->get<SnGroup>(2);
	SnGroup* gRHand = gCharacter->get<SnGroup>(3);
	SnGroup* gLLeg = gCharacter->get<SnGroup>(4);
	SnGroup* gRLeg = gCharacter->get<SnGroup>(5);
	GsMat rotation, toLocal, toOrigin;
	GsBox b[6];
	for (int i = 0; i < 6; i++) {
		gCharacter->get<SnGroup>(i)->get<SnModel>(1)->get_bounding_box(b[i]);
	}

	switch (e.key)
	{
	case GsEvent::KeyEsc: gs_exit(); return 1;
	case 'n': { bool b = !_nbut->value(); _nbut->value(b); show_normals(b); return 1; }
			//rotates head in positive direction
	case 'q': // same action as 'r'
	case 'r': {
		if (angle[0] > GS_PI / 6.0) {
			return 1;
		}
		//rotate head ccw
		//GsMat rotation;
		angle[0] += GS_PI / 8.0;
		rotation.roty((float)GS_PI / 8.0f);//rotate ccw
		gHead->get<SnTransform>(0)->get().mult(gHead->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'e': //same action as f
	case 'f': {
		if (angle[0] < GS_PI / -6.0) {
			return 1;
		}
		//rotate head cw
		//GsMat rotation;
		angle[0] += GS_PI / -8.0;
		rotation.roty((float)GS_PI / -8.0f);//rotate cw
		gHead->get<SnTransform>(0)->get().mult(gHead->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 't': {
		//limit the rotational angle to be from [pi,-pi/6]
		if (angle[2] < -GS_PI) {
			return 1;
		}
		//rotate left arm up
		//GsMat rotation, toLocal, toOrigin;
		angle[2] += GS_PI / -6.0;
		rotation.rotx((float)GS_PI / -6.0f);//rotate ccw
		toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
		rotation = toOrigin.inverse() * rotation * toOrigin;
		gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'g': {
		if (angle[2] > 0.0) {
			return 1;
		}
		//rotate left arm down
		angle[2] += GS_PI / 6.0;
		rotation.rotx((float)GS_PI / 6.0f);//rotate cw
		toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
		rotation = toOrigin.inverse() * rotation * toOrigin;
		gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'y': {
		if (angle[3] < -GS_PI) {
			return 1;
		}
		//rotate right arm up
		angle[3] += GS_PI / -6.0;
		rotation.rotx((float)GS_PI / -6.0f);//rotate ccw
		toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
		rotation = toOrigin.inverse() * rotation * toOrigin;
		gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'h': {
		if (angle[3] > 0.0) {
			return 1;
		}
		//rotate right arm down
		angle[3] += GS_PI / 6.0;
		rotation.rotx((float)GS_PI / 6.0f);//rotate cw
		toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
		rotation = toOrigin.inverse() * rotation * toOrigin;
		gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'u': {
		//index 4, cap when parallel to floor plane
		if (angle[4] < GS_PI / -2.0) {
			return 1;
		}
		//rotate left leg up
		angle[4] += GS_PI / -6.0;
		rotation.rotx((float)GS_PI / -6.0f);//rotate ccw
		toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
		rotation = toOrigin.inverse() * rotation * toOrigin;
		gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'j': {
		if (angle[4] > 0.0) {
			return 1;
		}
		//rotate left leg down
		angle[4] += GS_PI / 6.0;
		rotation.rotx((float)GS_PI / 6.0f);//rotate cw
		toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
		rotation = toOrigin.inverse() * rotation * toOrigin;
		gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'i': {
		if (angle[5] < GS_PI / -2.0) {
			return 1;
		}
		//rotate right leg up
		angle[5] += GS_PI / -6.0;
		rotation.rotx((float)GS_PI / -6.0f);//rotate ccw
		toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
		rotation = toOrigin.inverse() * rotation * toOrigin;
		gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'k': {
		//rotate right leg down
		if (angle[5] > 0.0) {
			return 1;
		}
		angle[5] += GS_PI / 6.0;
		rotation.rotx((float)GS_PI / 6.0f);//rotate cw
		toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
		rotation = toOrigin.inverse() * rotation * toOrigin;
		gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'w':
	case GsEvent::KeyUp: {
		if (movingForward == false) {
			//strideNum+=6;
			movingForward = true;
		}
		//consider resetting all body part positions and then animate.
		//will move left leg out and right leg back on stride 1
		GsMat movement;
		movement.translation(GsVec(0.0f, 0.0f, 0.1f));
		rootg()->get<SnGroup>(1)->get<SnTransform>(0)->get().mult((rootg()->get<SnGroup>(1))->get<SnTransform>(0)->get(), movement);//move body forward
		if (strideNum % 4 == 0) {//left leg up, right leg down, left arm down, right arm up
			//rotate left leg up
			angle[4] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);

			//rotate right leg down
			angle[5] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);

			//left arm down
			angle[2] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
			//right arm up
			angle[3] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		}
		else if (strideNum % 4 == 1) {
			//move forward and reset body positions
			//left leg down, right leg up, left arm up, right arm down
			//rotate left leg down
			angle[4] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);

			//rotate right leg up
			angle[5] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);

			//left arm up
			angle[2] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
			//right arm down
			angle[3] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		}
		else if (strideNum % 4 == 2) {
			//right leg forward, left leg back, right arm back, left arm forward
			//rotate left leg back
			angle[4] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);

			//rotate right leg forward
			angle[5] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);

			//left arm forward
			angle[2] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
			//right arm back
			angle[3] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		}
		else if (strideNum % 4 == 3) {
			//left leg forward, right leg back, left arm back, right arm forward
			//left leg forward
			angle[4] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);
			//right leg back
			angle[5] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);
			//left arm back
			angle[2] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
			//right arm forward
			angle[3] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		}
		strideNum++;
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 's':
	case GsEvent::KeyDown: {
		if (movingForward == true) {
			if (strideNum % 2 != 1) {
				strideNum += 2;
			}
			movingForward = false;
		}
		GsMat movement;
		movement.translation(GsVec(0.0f, 0.0f, -0.1f));
		(rootg()->get<SnGroup>(1))->get<SnTransform>(0)->get().mult((rootg()->get<SnGroup>(1))->get<SnTransform>(0)->get(), movement);
		if (strideNum % 4 == 0) {//left leg up, right leg down, left arm down, right arm up
			//rotate left leg up
			angle[4] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);

			//rotate right leg down
			angle[5] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);

			//left arm down
			angle[2] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
			//right arm up
			angle[3] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		}
		else if (strideNum % 4 == 1) {
			//move forward and reset body positions
			//left leg down, right leg up, left arm up, right arm down
			//rotate left leg down
			angle[4] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);

			//rotate right leg up
			angle[5] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);

			//left arm up
			angle[2] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
			//right arm down
			angle[3] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		}
		else if (strideNum % 4 == 2) {
			//right leg forward, left leg back, right arm back, left arm forward
			//rotate left leg back
			angle[4] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);

			//rotate right leg forward
			angle[5] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);

			//left arm forward
			angle[2] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
			//right arm back
			angle[3] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		}
		else if (strideNum % 4 == 3) {
			//left leg forward, right leg back, left arm back, right arm forward
			//left leg forward
			angle[4] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLLeg->get<SnTransform>(0)->get().mult(gLLeg->get<SnTransform>(0)->get(), rotation);
			//right leg back
			angle[5] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[4].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRLeg->get<SnTransform>(0)->get().mult(gRLeg->get<SnTransform>(0)->get(), rotation);
			//left arm back
			angle[2] += GS_PI / 6.0;
			rotation.rotx((float)GS_PI / 6.0f);
			toOrigin.setc4(GsVec(b[1].dx() / 2, -1 * b[2].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gLHand->get<SnTransform>(0)->get().mult(gLHand->get<SnTransform>(0)->get(), rotation);
			//right arm forward
			angle[3] += GS_PI / -6.0;
			rotation.rotx((float)GS_PI / -6.0f);
			toOrigin.setc4(GsVec(-1 * b[1].dx() / 2, -1 * b[3].dy() / 2, 0.0f));
			rotation = toOrigin.inverse() * rotation * toOrigin;
			gRHand->get<SnTransform>(0)->get().mult(gRHand->get<SnTransform>(0)->get(), rotation);
		}
		strideNum++;
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'a':
	case GsEvent::KeyLeft: {
		GsMat movement;
		movement.roty((float)GS_PI / 8.0f);
		(rootg()->get<SnGroup>(1))->get<SnTransform>(0)->get().mult((rootg()->get<SnGroup>(1))->get<SnTransform>(0)->get(), movement);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case 'd':
	case GsEvent::KeyRight: {
		GsMat movement;
		movement.roty((float)GS_PI / -8.0f);
		(rootg()->get<SnGroup>(1))->get<SnTransform>(0)->get().mult((rootg()->get<SnGroup>(1))->get<SnTransform>(0)->get(), movement);
		render(); // notify it needs redraw
		ws_check(); // redraw now
		return 1;
	}
	case GsEvent::KeySpace: {
		run_animation();
		cam = true;
		return 1;
	}
	default: gsout << "Key pressed: " << e.key << gsnl;
	}
	return 0;
}

int MyViewer::uievent ( int e )
{
	switch ( e )
	{	case EvNormals: show_normals(_nbut->value()); return 1;
		case EvAnimate: run_animation(); return 1;
		case EvExit: gs_exit();
	}
	return WsViewer::uievent(e);
}
