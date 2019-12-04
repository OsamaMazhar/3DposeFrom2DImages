
#declare application components

declare_PID_Component(
  APPLICATION
  NAME hello-app
  DIRECTORY hello
)

declare_PID_Component_Dependency(
  COMPONENT hello-app
  NATIVE hello-static
)
