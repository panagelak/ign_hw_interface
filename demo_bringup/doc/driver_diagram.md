@startuml driver_diagram
skinparam componentStyle uml2
skinparam defaultTextAlignment center

left to right direction

package "Ignition Hardware Interface" as gnHW <<Folder>> {
  abstract class  "HWBase" as base{
      +virtual read()
      +update()
      +virtual write()
  }
  interface  "IgnitionHWInterface" as interface{
      +read()
      +write()
  }
  class      "HWControlLoop" as cloop{
      +run()
      -update()
  }
}

package "Ignition bridge" as gnDriver <<Folder>> {
  class "Ros Topic Interface" as ignBridge{
  +pubJointStates()
  -cmdJoint() x6
  }
}

package "Ignition plugins" as ignPlugins <<Folder>> {
  abstract class  "ignition plugins" as ign_plugins{
      +joint-state-publisher
      -joint-position-controller x 6 
  }
}



' Connections between classes

' Inside ignition hardware
base  +--> interface
interface -up-> cloop

' Hardware Interface to ignBridge
interface --> ignBridge
ignBridge --> interface
' ignBridge TO ignPlugins
ignBridge *--> ignPlugins
ignPlugins *--> ignBridge

@enduml