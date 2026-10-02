#include <GL/glut.h>
#include <math.h>
#include <cstdio>
#include <unistd.h>
#include <string>
#include <algorithm>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "its_msgs/msg/soft_contact_sensing_problem_solution_stamped.hpp"

#include "its_ros2/Fingertip.hpp"   // adjust to your include path

using namespace fingertip;

// ============================================================================
//  Globals
// ============================================================================
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Subscription<its_msgs::msg::SoftContactSensingProblemSolutionStamped>::SharedPtr g_sub = nullptr;

static Surface g_surface;
static bool    g_use_mesh = false;

double A, B, C;
double force_th;

enum class Theme : unsigned short int { Dark = 1, Light = 2 };
Theme theme    = Theme::Dark;
int  changingcolor = 0;
bool fullscreen    = false;
bool mouseDown     = false;

float xrot = 0.0f, yrot = 0.0f;
float xdiff = 0.0f, ydiff = 0.0f;
float x_pos = 0, y_pos = 0, z_pos = 0, d_def = 0;
float fn[3] = {}, ft[3] = {};
float lt = 0;

// --- camera ---
float zoom      = 6.0f;
float pan_x     = 0.0f;
float pan_y     = 0.0f;
bool  midDown   = false;
float midXstart = 0, midYstart = 0;
float panXstart = 0, panYstart = 0;

// --- HUD visibility ---
bool show_help   = true;   // 'h' toggles controls legend
bool show_status = true;   // 'a' toggles live data overlay

// ============================================================================
//  HUD helpers — draw 2-D text overlays in screen space
// ============================================================================

// Draw a filled, semi-transparent rectangle in screen coordinates.
// Call between glOrtho setup and text rendering.
static void drawHUDBackground(float x, float y, float w, float h,
                               float r, float g, float b, float a)
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glColor4f(r, g, b, a);
    glBegin(GL_QUADS);
    glVertex2f(x,     y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x,     y + h);
    glEnd();
    glDisable(GL_BLEND);
}

// Render a single line of bitmap text at (x, y) in screen coords.
static void drawHUDString(float x, float y,
                           float r, float g, float b,
                           const char* str)
{
    glColor3f(r, g, b);
    glRasterPos2f(x, y);
    for (const char* c = str; *c; ++c)
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
}

// Push an orthographic projection that maps 1:1 to window pixels,
// origin bottom-left.  Returns window width/height via out-params.
static void beginHUD(int& W, int& H)
{
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    W = vp[2]; H = vp[3];

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, W, 0, H);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
}

static void endHUD()
{
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// ============================================================================
//  HUD panels
// ============================================================================

// Controls / keybindings legend  (toggled with 'h')
static void drawHelpLegend(int /*W*/, int H)
{
    const int PAD   = 10;
    const int LH    = 16;   // line height in pixels
    const int LINES = 10;
    const int PW    = 230;
    const int PH    = PAD * 2 + LH * LINES;

    float bx = (float)PAD;
    float by = (float)(H - PAD - PH);

    // Background panel
    drawHUDBackground(bx, by, (float)PW, (float)PH,
                      0.05f, 0.05f, 0.05f, 0.65f);

    // Border
    glDisable(GL_LIGHTING);
    glColor4f(0.6f, 0.6f, 0.6f, 0.8f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(bx,      by);
    glVertex2f(bx + PW, by);
    glVertex2f(bx + PW, by + PH);
    glVertex2f(bx,      by + PH);
    glEnd();

    // Title
    float tx = bx + PAD;
    float ty = by + PH - PAD - LH;
    drawHUDString(tx, ty, 1.0f, 0.85f, 0.25f, "=== Controls ===");
    ty -= LH;

    // Entries
    struct Entry { const char* key; const char* desc; };
    static const Entry entries[] = {
        { "Left drag",    "Orbit"          },
        { "Mid drag",     "Pan"            },
        { "Scroll",       "Zoom in / out"  },
        { "F1",           "Toggle fullscreen" },
        { "H",            "Toggle controls" },
        { "A",            "Toggle status"  },
        { "Esc",          "Quit"           },
    };

    for (const auto& e : entries) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%-12s %s", e.key, e.desc);
        drawHUDString(tx, ty, 0.85f, 0.85f, 0.85f, buf);
        ty -= LH;
    }
}

// Live contact-state data overlay  (toggled with 'a')
static void drawStatusOverlay(int W, int H,
                               float x_pos, float y_pos, float z_pos,
                               const float fn[3], const float ft[3],
                               float lt, float d_def,
                               float zoom, float pan_x, float pan_y)
{
    const int PAD   = 10;
    const int LH    = 16;
    const int LINES = 11;
    const int PW    = 260;
    const int PH    = PAD * 2 + LH * LINES;

    float bx = (float)(W - PAD - PW);
    float by = (float)(H - PAD - PH);

    drawHUDBackground(bx, by, (float)PW, (float)PH,
                      0.05f, 0.05f, 0.05f, 0.65f);

    glColor4f(0.6f, 0.6f, 0.6f, 0.8f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(bx,      by);
    glVertex2f(bx + PW, by);
    glVertex2f(bx + PW, by + PH);
    glVertex2f(bx,      by + PH);
    glEnd();

    float tx = bx + PAD;
    float ty = by + PH - PAD - LH;
    char buf[64];

    drawHUDString(tx, ty, 1.0f, 0.85f, 0.25f, "=== Contact state ===");
    ty -= LH;

    double Fn = sqrt((double)fn[0]*fn[0] + (double)fn[1]*fn[1] + (double)fn[2]*fn[2]);
    double Ft = sqrt((double)ft[0]*ft[0] + (double)ft[1]*ft[1] + (double)ft[2]*ft[2]);

    snprintf(buf, sizeof(buf), "PoC  (mm): %6.2f %6.2f %6.2f",
             (double)x_pos*10, (double)y_pos*10, (double)z_pos*10);
    drawHUDString(tx, ty, 0.85f, 0.85f, 0.85f, buf); ty -= LH;

    snprintf(buf, sizeof(buf), "Fn   (N) : %6.3f", Fn);
    drawHUDString(tx, ty, 0.9f, 0.5f, 0.5f, buf); ty -= LH;

    snprintf(buf, sizeof(buf), "fn dir   : %5.2f %5.2f %5.2f",
             Fn > 1e-6 ? fn[0]/Fn : 0.0, Fn > 1e-6 ? fn[1]/Fn : 0.0, Fn > 1e-6 ? fn[2]/Fn : 0.0);
    drawHUDString(tx, ty, 0.85f, 0.85f, 0.85f, buf); ty -= LH;

    snprintf(buf, sizeof(buf), "Ft   (N) : %6.3f", Ft);
    drawHUDString(tx, ty, 0.5f, 0.9f, 0.5f, buf); ty -= LH;

    snprintf(buf, sizeof(buf), "ft vec   : %5.2f %5.2f %5.2f",
             (double)ft[0], (double)ft[1], (double)ft[2]);
    drawHUDString(tx, ty, 0.85f, 0.85f, 0.85f, buf); ty -= LH;

    snprintf(buf, sizeof(buf), "Torque   : %6.3f", (double)lt);
    drawHUDString(tx, ty, 0.7f, 0.7f, 1.0f, buf); ty -= LH;

    snprintf(buf, sizeof(buf), "Deform   : %6.3f mm", (double)d_def*10);
    drawHUDString(tx, ty, 0.85f, 0.85f, 0.85f, buf); ty -= LH;

    snprintf(buf, sizeof(buf), "Zoom     : %5.2f", (double)zoom);
    drawHUDString(tx, ty, 0.75f, 0.75f, 0.75f, buf); ty -= LH;

    snprintf(buf, sizeof(buf), "Pan      : %5.2f %5.2f", (double)pan_x, (double)pan_y);
    drawHUDString(tx, ty, 0.75f, 0.75f, 0.75f, buf);

    // Small "press A to hide" hint at panel bottom
    ty = by + 4;
    drawHUDString(tx, ty, 0.45f, 0.45f, 0.45f, "A - hide");
}

// Tiny corner hints shown when a panel is hidden
static void drawHiddenHint(int W, int H)
{
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    char buf[64];
    if (!show_help) {
        snprintf(buf, sizeof(buf), "H - show controls");
        drawHUDString(10.0f, (float)(H - 20), 0.45f, 0.45f, 0.45f, buf);
    }
    if (!show_status) {
        snprintf(buf, sizeof(buf), "A - show status");
        // right-align roughly
        drawHUDString((float)(W - 130), (float)(H - 20), 0.45f, 0.45f, 0.45f, buf);
    }
}

// ============================================================================
//  Drawing: mesh
// ============================================================================
void drawMesh(bool transparency) {
    if (g_surface.mesh.empty()) return;

    float mcolor[4] = {0.6f, 0.7f, 0.9f, transparency ? 0.15f : 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mcolor);

    glPushMatrix();
    glScalef(0.1f, 0.1f, 0.1f);
    glBegin(GL_TRIANGLES);
    for (const auto& tri : g_surface.mesh.triangles) {
        const Eigen::Vector3d n = tri.normal();
        glNormal3d(n.x(), n.y(), n.z());
        glVertex3d(tri.v0.x(), tri.v0.y(), tri.v0.z());
        glVertex3d(tri.v1.x(), tri.v1.y(), tri.v1.z());
        glVertex3d(tri.v2.x(), tri.v2.y(), tri.v2.z());
    }
    glEnd();
    glPopMatrix();
}

// ============================================================================
//  Drawing: ellipsoid fallback
// ============================================================================
void drawEllipsoid(float a, float b, float c, int lats, int longs,
                   bool transparency = false) {
    float mcolor[4] = {0.8f, 0.8f, 0.8f, transparency ? 0.10f : 1.0f};
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mcolor);

    for (int i = lats/2; i <= lats; ++i) {
        float lat0 = (float)M_PI * (-0.5f + (float)(i-1)/lats);
        float z0 = sinf(lat0), zr0 = cosf(lat0);
        float lat1 = (float)M_PI * (-0.5f + (float)i/lats);
        float z1 = sinf(lat1), zr1 = cosf(lat1);
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= longs; ++j) {
            float lng = 2*(float)M_PI * (float)(j-1)/longs;
            float vx = cosf(lng), vy = sinf(lng);
            glNormal3f(vx*zr0, vy*zr0, z0); glVertex3f(vx*zr0*a, vy*zr0*b, z0*c);
            glNormal3f(vx*zr1, vy*zr1, z1); glVertex3f(vx*zr1*a, vy*zr1*b, z1*c);
        }
        glEnd();
    }
}

// ============================================================================
//  Drawing: Circular Cylinder (1 infinite axis)
// ============================================================================
void drawCylinder(float r, int inf_axis, float length, int segments, bool transparency) {
    float mcolor[4] = {0.8f, 0.8f, 0.8f, transparency ? 0.10f : 1.0f};
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mcolor);

    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * (float)M_PI * (float)i / segments;
        float cosA = cosf(angle);
        float sinA = sinf(angle);

        if (inf_axis == 2) { // Z is infinite
            glNormal3f(cosA, sinA, 0.0f);
            glVertex3f(r * cosA, r * sinA, -length);
            glVertex3f(r * cosA, r * sinA,  length);
        } else if (inf_axis == 1) { // Y is infinite
            glNormal3f(cosA, 0.0f, sinA);
            glVertex3f(r * cosA, -length, r * sinA);
            glVertex3f(r * cosA,  length, r * sinA);
        } else { // X is infinite
            glNormal3f(0.0f, cosA, sinA);
            glVertex3f(-length, r * cosA, r * sinA);
            glVertex3f( length, r * cosA, r * sinA);
        }
    }
    glEnd();
}
// ============================================================================
//  Drawing: axis, torque, light
// ============================================================================
void drawAxis_v2() {
    int len = 1, dispX = -2, dispY = -2;
    glRotated(-yrot, 0,1,0); glRotated(-xrot, 1,0,0);
    glTranslated(dispX, dispY, 0);
    glRotated(xrot, 1,0,0); glRotated(yrot, 0,1,0);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(len,0,0);
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,len,0);
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,len);
    glEnd();
    glRotated(-yrot, 0,1,0); glRotated(-xrot, 1,0,0);
    glTranslated(-dispX, -dispY, 0);
    glRotated(xrot, 1,0,0); glRotated(yrot, 0,1,0);
}

void drawTorque(float lt) {
    float fRadius = 0.2f, fPrecision = 0.05f;
    float fCenterX = 0, fCenterY = 0, fAngle, fX = 0, fY = 0;
    glBegin(GL_LINE_STRIP);
    for (fAngle = 0; fAngle <= fabs(0.05f*lt*(float)M_PI); fAngle += fPrecision) {
        fX = fCenterX + fRadius * sinf(fAngle);
        fY = fCenterY + fRadius * cosf(fAngle);
        if (lt > 0) fX = -fX;
        glVertex3f(fX, fY, 0);
    }
    glEnd();
    glBegin(GL_TRIANGLES);
    glVertex3f(fX, fY+0.06f, 0); glVertex3f(fX, fY-0.06f, 0); glVertex3f(fX+0.06f, fY, 0);
    glEnd();
}

void light() {
    glEnable(GL_LIGHTING);
    GLfloat specular[]     = {1,1,1,1};
    GLfloat ambientLight[] = {0.2f,0.2f,0.2f,1};
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    GLfloat position[] = {0,0,5,1};
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    glLightfv(GL_LIGHT1, GL_POSITION, position);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
    glEnable(GL_LIGHT1);
}

// ============================================================================
//  GLUT callbacks
// ============================================================================
bool init() {
    switch (theme) {
        case Theme::Dark:  glClearColor(0.15f,0.15f,0.15f,0); break;
        default:           glClearColor(0.93f,0.93f,0.93f,0); break;
    }
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glClearDepth(1.0f);
    return true;
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0, 0, zoom,
              pan_x, pan_y, 0,
              0, 1, 0);
    glRotatef(xrot,1,0,0); glRotatef(yrot,0,1,0);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LEQUAL);

    drawAxis_v2();
    light();

    // ---- fingertip surface --------------------------------------------------
    if (g_use_mesh) {
        drawMesh(false);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        drawMesh(true);
        glDisable(GL_BLEND);
    } else {
        if ( C < 0.0 ){
            drawCylinder((float)(A-d_def), 2, 1000.0, 30, false);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            drawCylinder((float)(A), 2, 1000.0, 30, true);
            glDisable(GL_BLEND);
        } else{
            drawEllipsoid((float)(A-d_def),(float)(B-d_def),(float)(C-d_def), 15, 30, false);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            drawEllipsoid((float)A,(float)B,(float)C, 15, 30, true);
            glDisable(GL_BLEND);
        }
    }

    glDisable(GL_DEPTH_TEST);

    // ---- contact point + force arrow ----------------------------------------
    if (changingcolor == 100) changingcolor = 0;
    ++changingcolor;
    float ecolor[] = {(float)(0.1*changingcolor/10.0), 0, (float)(1-0.1*changingcolor/10.0), 0.8f};
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, ecolor);
    glTranslated(x_pos, y_pos, z_pos);

    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1,0.3f,0.3f); glVertex3f(0,0,0); glVertex3f(fn[0]/10,fn[1]/10,fn[2]/10);
    glEnd();
    glBegin(GL_LINES);
    glColor3f(0.3f,1,0.3f); glVertex3f(0,0,0); glVertex3f(ft[0]/10,ft[1]/10,ft[2]/10);
    glEnd();
    glEnable(GL_LIGHTING);

    double f_tx=fn[0]+ft[0], f_ty=fn[1]+ft[1], f_tz=fn[2]+ft[2];
    double Ftot = sqrt(f_tx*f_tx + f_ty*f_ty + f_tz*f_tz);
    float theta1 = (float)(180-(180/M_PI)*atan2(f_tz,f_tx));
    float theta2 = (float)((180/M_PI)*atan2(f_ty,sqrt(f_tx*f_tx+f_tz*f_tz)));
    glRotated(90,0,1,0); glRotated(theta1,0,1,0); glRotated(theta2,1,0,0);

    if (Ftot > force_th) {
        static GLUquadricObj* q = nullptr;
        if (!q) { q = gluNewQuadric(); gluQuadricNormals(q, GLU_SMOOTH); }
        float ecolor2[] = {0.9f,0.6f,0.1f,0.9f};
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, ecolor2);
        gluCylinder(q, 0.01, (float)(0.05*Ftot), (float)(Ftot/3), 30, 20);
        drawTorque(lt);
    }

    glDisable(GL_LIGHTING);

    // ---- HUD ----------------------------------------------------------------
    int W, H;
    beginHUD(W, H);

    if (show_help)
        drawHelpLegend(W, H);

    if (show_status)
        drawStatusOverlay(W, H,
                          x_pos, y_pos, z_pos,
                          fn, ft, lt, d_def,
                          zoom, pan_x, pan_y);

    // show tiny corner hints for whichever panel(s) are hidden
    if (!show_help || !show_status)
        drawHiddenHint(W, H);

    endHUD();

    glFlush();
    glutSwapBuffers();
    glutPostRedisplay();
}

void resize(int w, int h) {
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    glViewport(0,0,w,h);
    gluPerspective(45,(float)w/h,1,100);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
}

void idle() { if (g_node) rclcpp::spin_some(g_node); usleep(10000); }

void keyboard(unsigned char key, int, int) {
    switch (key) {
        case 27:                        // Esc — quit
            rclcpp::shutdown(); exit(0);
        case 'h': case 'H':             // toggle controls legend
            show_help = !show_help;
            glutPostRedisplay();
            break;
        case 'a': case 'A':             // toggle live-data overlay
            show_status = !show_status;
            glutPostRedisplay();
            break;
        default: break;
    }
}

void specialKeyboard(int key, int, int) {
    if (key == GLUT_KEY_F1) {
        fullscreen = !fullscreen;
        if (fullscreen) glutFullScreen();
        else { glutReshapeWindow(500,500); glutPositionWindow(50,50); }
    }
}

void mouse(int button, int state, int x, int y) {
    // scroll wheel — zoom
    if (button == 3 && state == GLUT_DOWN) {
        zoom = std::max(1.0f, zoom - 0.5f);
        glutPostRedisplay(); return;
    }
    if (button == 4 && state == GLUT_DOWN) {
        zoom = std::min(50.0f, zoom + 0.5f);
        glutPostRedisplay(); return;
    }

    // left button — orbit
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        mouseDown = true; xdiff = x - yrot; ydiff = -y + xrot;
    } else if (button == GLUT_LEFT_BUTTON) {
        mouseDown = false;
    }

    // middle button — pan
    if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) {
        midDown   = true;
        midXstart = (float)x; midYstart = (float)y;
        panXstart = pan_x;    panYstart = pan_y;
    } else if (button == GLUT_MIDDLE_BUTTON) {
        midDown = false;
    }
}

void mouseMotion(int x, int y) {
    if (mouseDown) { yrot = x - xdiff; xrot = y + ydiff; glutPostRedisplay(); }
    if (midDown) {
        pan_x = panXstart + (x - midXstart) * 0.01f * zoom;
        pan_y = panYstart - (y - midYstart) * 0.01f * zoom;
        glutPostRedisplay();
    }
}

// ============================================================================
//  ROS 2 callback
// ============================================================================
void Callback_contactstate(
    const its_msgs::msg::SoftContactSensingProblemSolutionStamped::SharedPtr msg)
{
    x_pos = (float)(msg->csp.c.x / 10.0);
    y_pos = (float)(msg->csp.c.y / 10.0);
    z_pos = (float)(msg->csp.c.z / 10.0);
    d_def = (float)(msg->d    / 10.0);
    double Fn = msg->csp.fn;
    fn[0]=(float)(msg->csp.n.x*Fn); fn[1]=(float)(msg->csp.n.y*Fn); fn[2]=(float)(msg->csp.n.z*Fn);
    ft[0]=(float)msg->csp.ft.x; ft[1]=(float)msg->csp.ft.y; ft[2]=(float)msg->csp.ft.z;
    lt   =(float)msg->csp.t;
    glutPostRedisplay();
}

// ============================================================================
//  Main
// ============================================================================
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("soft_csp_viz");

    g_node->declare_parameter("fingertip.principalSemiAxis.a", 15.0);
    g_node->declare_parameter("fingertip.principalSemiAxis.b", 15.0);
    g_node->declare_parameter("fingertip.principalSemiAxis.c", 15.0);
    g_node->declare_parameter("algorithm.force_threshold",      0.5);
    g_node->declare_parameter("soft_viz.theme",                 "Dark");
    g_node->declare_parameter("fingertip.id",                   "myFinger");
    g_node->declare_parameter("fingertip.mesh.filepath",        "");
    g_node->declare_parameter("fingertip.mesh.type",            "obj");
    g_node->declare_parameter("fingertip.mesh.scale",           1.0);
    g_node->declare_parameter("soft_viz.mesh.filepath",        "");

    A        = g_node->get_parameter("fingertip.principalSemiAxis.a").as_double() / 10.0;
    B        = g_node->get_parameter("fingertip.principalSemiAxis.b").as_double() / 10.0;
    C        = g_node->get_parameter("fingertip.principalSemiAxis.c").as_double() / 10.0;
    force_th = g_node->get_parameter("algorithm.force_threshold").as_double();

    std::string theme_str = g_node->get_parameter("soft_viz.theme").as_string();
    theme = (theme_str == "Dark") ? Theme::Dark : Theme::Light;

    const std::string finger_id  = g_node->get_parameter("fingertip.id").as_string();

    const std::string its_mesh = g_node->get_parameter("fingertip.mesh.filepath").as_string();
    const std::string viz_mesh = g_node->get_parameter("soft_viz.mesh.filepath").as_string();
    const std::string mesh_file = (!viz_mesh.empty()) ? viz_mesh : its_mesh;
    const std::string mesh_type  = g_node->get_parameter("fingertip.mesh.type").as_string();
    const double      mesh_scale = g_node->get_parameter("fingertip.mesh.scale").as_double();

    if (!mesh_file.empty()) {
        if (mesh_type == "obj") {
            g_surface.surfaceType = SurfaceType::Mesh;
            g_use_mesh = g_surface.mesh.load(mesh_file, mesh_scale);
            RCLCPP_INFO(g_node->get_logger(), "Rendering mesh from: %s", mesh_file.c_str());
        } else {
            g_surface.surfaceType = SurfaceType::ConvexHull;
            g_surface.mesh.load(mesh_file, mesh_scale);
            g_surface.mesh.buildConvexHull();
            g_use_mesh = !g_surface.mesh.empty();
            RCLCPP_INFO(g_node->get_logger(), "Rendering convex hull from: %s", mesh_file.c_str());
        }
        if (!g_use_mesh)
            RCLCPP_WARN(g_node->get_logger(), "Mesh load failed, falling back to ellipsoid.");
    }

    g_sub = g_node->create_subscription<its_msgs::msg::SoftContactSensingProblemSolutionStamped>(
        "soft_csp_" + finger_id + "/solution", 100, Callback_contactstate);

    std::string title = "SoftCSPViz - " + finger_id;
    glutInit(&argc, argv);
    glutInitWindowPosition(50,50);
    glutInitWindowSize(500,500);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow(title.c_str());
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotion);
    glutReshapeFunc(resize);
    glutIdleFunc(idle);

    if (!init()) { rclcpp::shutdown(); return 1; }
    glutMainLoop();
    rclcpp::shutdown();
    return 0;
}
