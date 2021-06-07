
#ifndef QTOPENGL_INSECTBOT_H
#define QTOPENGL_INSECTBOT_H

namespace argos {
   class CQTOpenGLInsectbot;
   class CInsectbotEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLInsectbot {

   public:

      CQTOpenGLInsectbot();

      virtual ~CQTOpenGLInsectbot();

      virtual void Draw(CInsectbotEntity& c_entity);

   protected:

      /** Renders a materialless wheel
          - centered in 0,0,0
          - rotation axis: Y
       */
      void MakeWheel();

      /** Sets a white plastic material */
      void SetWhitePlasticMaterial();
      /** Sets a black tire material */
      void SetBlackTireMaterial();
      /** Sets a circuit board material */
      void SetCircuitBoardMaterial();
      /** Renders the wheels */
      void RenderWheel();
      /** Renders the base (apart from the wheels) */
      void RenderBase();


   private:

      /** Start of the display list index */
      GLuint m_unLists;

      /** List corresponding to the materialless wheel */
      GLuint m_unBasicWheelList;

      /** insectbot wheel */
      GLuint m_unWheelList;
      /** insectbot base module */
      GLuint m_unBaseList;
      /** insectbot LED */
      GLuint m_unLEDList;

      /** Number of vertices to display the round parts
          (chassis, etc.) */
      GLuint m_unVertices;

   };

}

#endif
