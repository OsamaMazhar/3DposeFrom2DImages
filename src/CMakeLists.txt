
#declare library components
declare_PID_Component(
  STATIC_LIB
  NAME hello-static
  DIRECTORY hello
)

declare_PID_Component(
  SHARED_LIB
  NAME hello-shared
  DIRECTORY hello
  INTERNAL DEFINITIONS DO_NOT_PRINT_HELLO
)
