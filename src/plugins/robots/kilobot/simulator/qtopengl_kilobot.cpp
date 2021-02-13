/**
 * @file <argos3/plugins/robots/kilobot/simulator/qtopengl_kilobot.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#include "qtopengl_kilobot.h"
#include "kilobot_measures.h"
#include "kilobot_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* All measures are in meters */


   /****************************************/
   /****************************************/

   CQTOpenGLKilobot::CQTOpenGLKilobot() :
      m_unVertices(24) {
      /* Reserve the needed display lists */
      m_unLists = glGenLists(1);

      /* Assign indices for better referencing (later) */
      // m_unBasicWheelList            = m_unLists;
      // m_unWheelList                 = m_unLists + 1;
      m_unBaseList                  = m_unLists + 0;
      // m_unLEDList                   = m_unLists + 3;


      /* Create the base module display list */
      glNewList(m_unBaseList, GL_COMPILE);
      RenderBase();
      glEndList();

   }

   /****************************************/
   /****************************************/

   CQTOpenGLKilobot::~CQTOpenGLKilobot() {
      glDeleteLists(m_unLists, 1);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLKilobot::Draw(CKilobotEntity& c_entity) {
      glPushMatrix();
      // glTranslatef(KILOBOT_ECCENTRICITY, 0.0f, 0.0f);
      glCallList(m_unBaseList);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   // void CQTOpenGLKilobot::SetWhitePlasticMaterial() {
   //    const GLfloat pfColor[]     = {   1.0f, 1.0f, 1.0f, 1.0f };
   //    const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
   //    const GLfloat pfShininess[] = { 100.0f                   };
   //    const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
   //    glMaterialfv(GL_SIDE_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
   //    glMaterialfv(GL_SIDE_AND_BACK, GL_SPECULAR,            pfSpecular);
   //    glMaterialfv(GL_SIDE_AND_BACK, GL_SHININESS,           pfShininess);
   //    glMaterialfv(GL_SIDE_AND_BACK, GL_EMISSION,            pfEmission);
   // }

   /****************************************/
   /****************************************/

   // void CQTOpenGLKilobot::SetCircuitBoardMaterial() {
   //    const GLfloat pfColor[]     = { 0.0f, 0.0f, 1.0f, 1.0f };
   //    const GLfloat pfSpecular[]  = { 0.5f, 0.5f, 1.0f, 1.0f };
   //    const GLfloat pfShininess[] = { 10.0f                  };
   //    const GLfloat pfEmission[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
   //    glMaterialfv(GL_SIDE_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
   //    glMaterialfv(GL_SIDE_AND_BACK, GL_SPECULAR,            pfSpecular);
   //    glMaterialfv(GL_SIDE_AND_BACK, GL_SHININESS,           pfShininess);
   //    glMaterialfv(GL_SIDE_AND_BACK, GL_EMISSION,            pfEmission);
   // }

   /****************************************/
   /****************************************/

   void CQTOpenGLKilobot::RenderBase() {
   glEnable(GL_COLOR_MATERIAL);
      /* Set material */
      // SetCircuitBoardMaterial();
   glTranslatef(0.0f, 0.0f, 0.005f);  // Move right and into the screen
   const float top = 0.02f;
   const float side = 0.01f;
   const float front = 0.02f;
   const float lside = -0.02f;
   glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
      // side face (y = 0.05f)
      // Define vertices in counter-clockwise (CCW) order with normal pointing out
      glColor3f(0.0f, 0.01f, 0.0f);
      glVertex3f( front, side, lside);
      glVertex3f(lside, side, lside);
      glVertex3f(lside, side,  top);
      glVertex3f( front, side,  top);
 
      // Bottom face (y = lside)
      glVertex3f( front, lside,  top);
      glVertex3f(lside, lside,  top);
      glVertex3f(lside, lside, lside);
      glVertex3f( front, lside, lside);
 
      // Side face  (z = top)
      glVertex3f( front,  side, top);
      glVertex3f(lside,  side, top);
      glVertex3f(lside, lside, top);
      glVertex3f( front, lside, top);
 
      // Back face (z = lside)
      glVertex3f( front, lside, lside);
      glVertex3f(lside, lside, lside);
      glVertex3f(lside,  side, lside);
      glVertex3f( front,  side, lside);
 
      // Left face (x = lside)
      glVertex3f(lside,  side,  top);
      glVertex3f(lside,  side, lside);
      glVertex3f(lside, lside, lside);
      glVertex3f(lside, lside,  top);
 
      // Right face (x = 0.05f)
      glVertex3f(front,  side, lside);
      glVertex3f(front,  side,  top);
      glVertex3f(front, lside,  top);
      glVertex3f(front, lside, lside);
   glEnd();  // End of drawing color-cube
   glColor3f(1.0f, 0.01f, 0.0f);
 
   }


   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawKilobotNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CKilobotEntity& c_entity) {
         static CQTOpenGLKilobot m_cModel;
         c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawKilobotSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CKilobotEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawKilobotNormal, CKilobotEntity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawKilobotSelected, CKilobotEntity);

   /****************************************/
   /****************************************/

}
