//Gazebo header for getting core gazebo functions
#include <gazebo/gazebo.hh>

//All gazebo plugins should have gazebo namespace

namespace gazebo
{

  //The custom WorldpluginTutorials is inheriting from standard worldPlugin. Each world plugin has to inheriting from standard plugin type. 

  class WorldPluginTutorial : public WorldPlugin
  {

    public: WorldPluginTutorial() : WorldPlugin()
            {
              printf("Hello World!\n");
            }


 //The Load function can receive the SDF elements 
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };

//Registering World Plugin with Simulator 
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
