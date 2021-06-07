
#include "qtopengl_kilobot.h"
#include "insectbot_measures.h"
#include "insectbot_entity.h"
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
      m_unBaseList                  = m_unLists + 0;


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

   void CQTOpenGLKilobot::Draw(CInsectbotEntity& c_entity) {
      glPushMatrix();
      glCallList(m_unBaseList);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLKilobot::RenderBase() {
   glEnable(GL_COLOR_MATERIAL);

   glTranslatef(-INSECTBOT_BASE_HEIGHT*(0.8)/2, 0.0f, 0.001f);
   const float height = INSECTBOT_BASE_WIDTH;
   const float side = INSECTBOT_BASE_WIDTH;
   const float length = INSECTBOT_BASE_HEIGHT*(0.8);
   const float lside = -INSECTBOT_BASE_WIDTH;
   glBegin(GL_QUADS);                // Begin drawing a cube with 6 quads
      // side face (y = 0.05f)
      glColor3f(0.0f, 0.01f, 0.0f);
      glVertex3f( length, side, lside);
      glVertex3f(lside, side, lside);
      glVertex3f(lside, side,  height);
      glVertex3f( length, side,  height);
 
      // Bottom face (y = lside)
      glVertex3f( length, lside,  height);
      glVertex3f(lside, lside,  height);
      glVertex3f(lside, lside, lside);
      glVertex3f( length, lside, lside);
 
      // Side face  (z = height)
      glVertex3f( length,  side, height);
      glVertex3f(lside,  side, height);
      glVertex3f(lside, lside, height);
      glVertex3f( length, lside, height);
 
      // Back face (z = lside)
      glVertex3f( length, lside, lside);
      glVertex3f(lside, lside, lside);
      glVertex3f(lside,  side, lside);
      glVertex3f( length,  side, lside);
 
      // Left face (x = lside)
      glVertex3f(lside,  side,  height);
      glVertex3f(lside,  side, lside);
      glVertex3f(lside, lside, lside);
      glVertex3f(lside, lside,  height);
 
      // Right face (x = length)
      glVertex3f(length,  side, lside);
      glVertex3f(length,  side,  height);
      glVertex3f(length, lside,  height);
      glVertex3f(length, lside, lside);
   glEnd();
   glColor3f(1.0f, 0.01f, 0.0f);
 
   }


   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawKilobotNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CInsectbotEntity& c_entity) {
         static CQTOpenGLKilobot m_cModel;
         c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawKilobotSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CInsectbotEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawKilobotNormal, CInsectbotEntity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawKilobotSelected, CInsectbotEntity);

   /****************************************/
   /****************************************/

}
