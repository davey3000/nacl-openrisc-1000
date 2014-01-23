/*
 * OpenRISC 1000 emulator top-level
 * 
 * Include the PPAPI-compatible module and instance classes for creating,
 * initializing and starting an instance of the emulator.
 */

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/mount.h>

#include "ppapi/c/pp_errors.h"
#include "ppapi/cpp/instance.h"
#include "ppapi/cpp/module.h"
#include "ppapi/cpp/var.h"
#include "nacl_io/nacl_io.h"

#include "cpu.hpp"

// The Instance class.  One of these exists for each instance of your NaCl
// module on the web page.  The browser will ask the Module object to create
// a new Instance for each occurrence of the <embed> tag that has these
// attributes:
//     src="hello_tutorial.nmf"
//     type="application/x-pnacl"
// To communicate with the browser, you must override HandleMessage() to
// receive messages from the browser, and use PostMessage() to send messages
// back to the browser.  Note that this interface is asynchronous.
class OREmulatorInstance : public pp::Instance {
private:
  CPU* cpu;

  pthread_t tid;
  
public:
  // The constructor creates the plugin-side instance.
  // @param[in] instance the handle to the browser-side plugin instance.
  explicit OREmulatorInstance(PP_Instance instance) : pp::Instance(instance) {
  }

  virtual ~OREmulatorInstance() {
    delete cpu;
  }

  virtual bool Init(uint32_t argc, const char* arg_n[], const char* argv[]) {
    // Create file system to hold the binaries that will be loaded into the
    // emulator
    nacl_io_init_ppapi(pp_instance(),
                       pp::Module::Get()->get_browser_interface());

    umount("/");
    mount("", "/", "memfs", 0, "");
    
    mount("bin", "/bin", "httpfs", 0, "");

    // Init the main emulator objects
    cpu = new CPU(this);
    
    // Create thread to load boot images
    if (pthread_create(&tid, NULL, LoadResourcesAndStart, this)) {
      fprintf(stderr, "ERROR: OREmulatorInstance.Init() failed to create pthread");
      return false;
    }

    return true;
  }

  static void* LoadResourcesAndStart(void* pInst) {
    OREmulatorInstance* orEmulator = (OREmulatorInstance*)pInst;

    // Load the Linux boot image
    if (!orEmulator->cpu->LoadRAMImage("/bin/vmlinux.bin.bz2")) {
      return NULL;
    }
    orEmulator->cpu->AnalyzeImage();

    // Load the HDD image
    if (!orEmulator->cpu->LoadHDDImage("/bin/hdd.bin.bz2")) {
      return NULL;
    }

    // Start the emulator
    // REVISIT: need a mechanism for adjusting the number of instructions executed with each Step() call based on the system speed
    do {
      orEmulator->cpu->Step(100000, 64);
    } while (true);

    return NULL;
  }

  // Handler for messages coming in from the browser via postMessage().  The
  // @a var_message can contain be any pp:Var type; for example int, string
  // Array or Dictinary. Please see the pp:Var documentation for more details.
  // @param[in] var_message The message posted by the browser.
  virtual void HandleMessage(const pp::Var& var_message) {
    /*if (!var_message.is_string())
      return;

    std::string message = var_message.AsString();
    pp::Var var_reply;
    if (message == kHelloString) {
      var_reply = pp::Var(kReplyString);
      PostMessage(var_reply);
    }*/

    // Send integer events from JS straight to the CPU
    if (!var_message.is_int()) {
      uint32_t msgInt = var_message.AsInt();

      cpu->OnExtMessage(msgInt);
    }
  }

  virtual void DidChangeView(const pp::View& view) {
    cpu->OnViewChange(view);
  }
};

// The Module class.  The browser calls the CreateInstance() method to create
// an instance of your NaCl module on the web page.  The browser creates a new
// instance for each <embed> tag with type="application/x-pnacl".
class OREmulatorModule : public pp::Module {
public:
  OREmulatorModule() : pp::Module() {
    srand(time(NULL));
  }

  virtual ~OREmulatorModule() {}

  // Create and return a OREmulatorInstance object.
  // @param[in] instance The browser-side instance.
  // @return the plugin-side instance.
  virtual pp::Instance* CreateInstance(PP_Instance instance) {
    return new OREmulatorInstance(instance);
  }
};

namespace pp {
  // Factory function called by the browser when the module is first loaded.
  // The browser keeps a singleton of this module.  It calls the
  // CreateInstance() method on the object you return to make instances.  There
  // is one instance per <embed> tag on the page.  This is the main binding
  // point for your NaCl module with the browser.
  Module* CreateModule() {
    return new OREmulatorModule();
  }

}  // namespace pp
