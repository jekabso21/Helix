#include <gdextension_interface.h>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "frame_publisher.hpp"

using namespace godot;

namespace {

void initialize_fpvsim(ModuleInitializationLevel level) {
  if (level != MODULE_INITIALIZATION_LEVEL_SCENE) {
    return;
  }
  GDREGISTER_CLASS(fpvsim::FramePublisher);
}

void uninitialize_fpvsim(ModuleInitializationLevel level) { (void)level; }

}  // namespace

extern "C" GDExtensionBool GDE_EXPORT
fpvsim_library_init(GDExtensionInterfaceGetProcAddress get_proc_address,
                    const GDExtensionClassLibraryPtr library, GDExtensionInitialization *init) {
  GDExtensionBinding::InitObject object(get_proc_address, library, init);
  object.register_initializer(initialize_fpvsim);
  object.register_terminator(uninitialize_fpvsim);
  object.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
  return object.init();
}
