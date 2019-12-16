
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

void MyViewer::add_model ( SnShape* s, GsVec p )
{

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

	tCharacter = new SnTransform;
	tCharacter->get().translation(GsVec(0.0f, (b[0].dy() / 2) + (b[1].dy()) + (b[4].dy()), 0.0f));
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
	SnGroup *eGroup, *floorGroup;
	SnPrimitive *floor;
	GsBox fBox;
	GsMat fMat;
	
	floor = new SnPrimitive(GsPrimitive::Box, 5.0f, 0.25f, 5.0f);
	floor->prim().material.diffuse = GsColor::green;
	floor->prim().get_bounding_box(fBox);

	SnTransform *floorTrans, *globalTrans;
	globalTrans = new SnTransform;
	floorTrans = new SnTransform;

	fMat.translation(0, fBox.dy()/-2.0f, 0);
	floorTrans->set(fMat);

	floorGroup = new SnGroup;
	floorGroup->separator(true);
	floorGroup->add(floorTrans);
	floorGroup->add(floor);

	eGroup = new SnGroup;
	eGroup->separator(true);
	eGroup->add(globalTrans);
	eGroup->add(floorGroup);

	SnGroup* houseG;
	SnModel* houseM;
	SnTransform* houseT;
	GsMat houseMat;
	GsBox houseB;

	houseM = new SnModel;
	//houseM->model()->load("../village_house/medieval house.3ds");
	//houseM->model()->G.push()->dmap->fname.set("../village_house/house2.png");
	//houseM->model()->get_bounding_box(houseB);
	//houseMat.translation(GsVec(0.0f, houseB.dy() / 2, -3.0f));
	houseT = new SnTransform;
	houseT->set(houseMat);

	houseG = new SnGroup;
	houseG->separator(true);
	houseG->add(houseT);
	houseG->add(houseM);

	eGroup->add(houseG);


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
	SnLines* path = new SnLines;//create path
	path->init();
	path->color(GsColor::yellow);
	path->line_width(3.5f);
	GsArray<GsPnt> controlPnts;//generate control points for curve
	//GsArray<GsPnt>* curvePnts;//will need to keep curvePnts
	SnGroup* pathG = new SnGroup;//will be a subgroup of environment
	pathG->separator(true);

	float height = 1.0f;
	controlPnts.push() = GsPnt(-2, height, 0);
	controlPnts.push() = GsPnt(0, height, -5);
	controlPnts.push() = GsPnt(-2, height, -5);
	float delta = 0.125f;

	path->begin_polyline();
	for (float t = 0; t < 1.0f; t += delta) {
		GsPnt pnt = eval_Bezier(t, controlPnts);
		path->push(pnt);//to display the curve
		//curvePnts->push(pnt);// to easily access the values of the curve points
	}
	path->end_polyline();
	pathG->add(path);
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
void MyViewer::build_scene ()
{
	// this project will have 3 main groups from root
	//0->environment 1->characters
	//environment sub groups: 
	buildEnvironment();//attatch egroup(separator true) to root. egroup consists of global transform, floor group(sep. true), house group(sep true).
	buildCharacter();//steve is index 1 from root. adds character group to root(sep. true). this group consists of global transform and body group(sep true)
	generatePaths();
	buildCars();
	//buildRobot();

}

// Below is an example of how to control the main loop of an animation:
void MyViewer::run_animation ()
{
	if ( _animating ) return; // avoid recursive calls
	_animating = true;
	
	double frdt = 1.0/30.0; // delta time to reach given number of frames per second
	double v = 4; // target velocity is 1 unit per second
	double t=0, lt=0, t0=gs_time();
	do // run for a while:
	{	while ( t-lt<frdt ) { ws_check(); t=gs_time()-t0; } // wait until it is time for next frame
		double yinc = (t-lt)*v;
		if ( t>2 ) yinc=-yinc; // after 2 secs: go down
		lt = t;
		if ( <0 ) ; // make sure it does not go below 0
		render(); // notify it needs redraw
		ws_check(); // redraw now
	}	while ( >0 );
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
		//change camera view from Birds Eye View to First Person View
		double lt, t0;
		if (fixedCamera == false) {
			//Bird Eye View Following Player

			camera().eye.y = b[0].dy() / 2 + b[1].dy() + b[4].dy() + 1.0f;
			camera().eye.x = rootg()->get<SnTransform>(1)->get().e14;
			camera().eye.z = rootg()->get<SnTransform>(1)->get().e34 - 5.0f;
			camera().center.x = rootg()->get<SnTransform>(1)->get().e14;
			camera().center.y = b[1].dy() + b[4].dy();
			camera().center.z = rootg()->get<SnTransform>(1)->get().e34;
			camera().up = cross(camera().center - camera().eye, GsVec::i);
			t0 = gs_time();
			do
			{
				lt = gs_time() - t0;
				camera().eye.y = b[0].dy() / 2 + b[1].dy() + b[4].dy() + 1.0f;
				camera().eye.x = rootg()->get<SnTransform>(1)->get().e14;
				camera().eye.z = rootg()->get<SnTransform>(1)->get().e34 - 5.0f;
				camera().center.x = rootg()->get<SnTransform>(1)->get().e14;
				camera().center.y = b[1].dy() + b[4].dy();
				camera().center.z = rootg()->get<SnTransform>(1)->get().e34;
				camera().up = cross(camera().center - camera().eye, GsVec::i);
				render();
				ws_check();
				message().setf("localtime = % f", lt);
			} while (lt < 10.0f);
		}
		else {
			//make fixed camera in the air
			camera().eye.x = 0.0;
			camera().eye.y = 4.0;
			camera().eye.z = -3.5;
			camera().center.x = 0;
			camera().center.y = 0.5;
			camera().center.z = 0;
			camera().up = cross(camera().center - camera().eye, GsVec::i);
			render();
			ws_check();
		}
		fixedCamera = !fixedCamera;
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
