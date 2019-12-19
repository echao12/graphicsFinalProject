# pragma once

# include <sig/sn_poly_editor.h>
# include <sig/sn_lines2.h>

# include <sigogl/ui_button.h>
# include <sigogl/ws_viewer.h>

// Viewer for this example:
class MyViewer : public WsViewer
{  protected :
	enum MenuEv { EvNormals, EvAnimate, EvExit };
	UiCheckButton* _nbut;
	bool _animating;
	bool fixedCamera;
	float camX, camY, camZ;
	double angle[6];
	int strideNum;
	bool movingForward;
	void buildCharacter();
	void buildEnvironment();
	void generatePaths();
	void buildCars();
	void moveCars();
	void buildRobot();
	void animateRobot();
	GsPnt eval_Bezier(float, const GsArray<GsPnt>& P);
   public :
	MyViewer ( int x, int y, int w, int h, const char* l );
	void build_ui ();
	void add_model ( SnShape* s, GsVec p );
	void addNPC( GsVec p);
	void spawnNPC();
	void build_scene ();
	void show_normals ( bool view );
	void run_animation ();
	void moveRobot();
	bool CarMoving;
	bool cam;
	virtual int handle_keyboard ( const GsEvent &e ) override;
	virtual int uievent ( int e ) override;
};

