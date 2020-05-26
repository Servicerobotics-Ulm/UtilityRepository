#ifndef _SMARTCOMPONENTBREAKPADCALLBACK_HH
#define _SMARTCOMPONENTBREAKPADCALLBACK_HH

static bool dumpCallback(const google_breakpad::MinidumpDescriptor& descriptor,void* context, bool succeeded) {
        printf("Component dumpCallback - minidump written to: %s\n", descriptor.path());


// use use this include the http_upload.cc in the breakpad_client.a
//Makefile.am
//203:
//      src/common/linux/http_upload.cc \
//  src/common/linux/http_upload.h
//call automake
//and make
//in component Cmake file, add LIST(APPEND USER_LIBS "dl")

//      if (succeeded == true) {
//              std::map<string, string> parameters;
//              std::map<string, string> files;
//              std::string proxy_host;
//              std::string proxy_userpasswd;
//
//              // See above for the URL section for how to generate the proper url for your server
//              std::string url("http://127.0.0.1:6097/post?format=minidump&token=57f2126dcef18bb0d2af35ec1d813f3775ee8228d1d886de522b2aedceff8b87");
//
//              // Add any attributes to the parameters map.
//              // Attributes such as uname.sysname, uname.version, cpu.count are
//              // extracted from minidump files automatically.
//              parameters["product_name"] = "SmartBreakPadTest";
//              parameters["version"] = "0.1.0";
//              // parameters["uptime"] uptime_sec;
//
//              files["upload_file_minidump"] = descriptor.path();
//
//              std::string response, error;
//              bool success = google_breakpad::HTTPUpload::SendRequest(url,
//                              parameters,
//                              files,
//                              proxy_host,
//                              proxy_userpasswd,
//                              "",
//                              &response,
//                              NULL,
//                              &error);
//      }

        return succeeded;
}


#endif
