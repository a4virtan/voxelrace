local action = _ACTION or ""
local boostlibs = "-lboost_system"
local boostinc = ""
local pkgconfig = "pkg-config"
local pkgconfigenv = ""
local pkgconfigpaths = ""
local libdir = ""
local polyvoxinc = ""
local polyvoxlibs = "-lPolyVoxCore"
local opengllibs = "-lGL -lGLU -lGLEW"

newaction {
  trigger = "prebuild",
  description = "Run code generators",
  execute = function ()
    os.execute("mkdir -p " ..  _WORKING_DIR .. "/Protocol")
    for i,file in pairs({ "ClientInfo", "Object", "ObjectUpdate", "RopeObject", "RopeObjectUpdate", "PlayerControl", "Player", "Property", "ServerInfo", "Team", "TournamentBegin", "TournamentEnd", "TournamentJoinRequest", "TournamentJoinResponse", "TournamentListRequest", "TournamentListResponse", "Tournament", "WorldUpdate", "ClientMsg", "ServerMsg", "CDMTypes" }) do
      os.execute("protoc --proto_path=" .. _WORKING_DIR .. "/protobuf --cpp_out=" .. _WORKING_DIR .. "/Protocol " .. _WORKING_DIR .. "/protobuf/" .. file.. ".proto")
    end
    os.execute("cd " .. _WORKING_DIR .. "/xml; ./xmllint.sh")
  end
}

newoption {
  trigger     = "with-clang",
  description = "Adjusts makefiles to work with LLVM + Clang",
}

newoption {
  trigger     = "with-boost",
  description = "Build with boost from specific location",
  value       = "PATH",
}

newoption {
  trigger     = "with-bullet",
  description = "Build with bullet from specific location",
  value       = "PATH",
}

newoption {
  trigger     = "with-ogre",
  description = "Build with OGRE from specific location",
  value       = "PATH",
}

newoption {
  trigger     = "with-polyvox",
  description = "Build with polyvox from specific location",
  value       = "PATH",
}

solution "VoxelRace"

  configurations { "Debug", "Release" }

  language "C++"
  kind "SharedLib"
  buildoptions { "--std=c++11 -Wall" }
  location ( "build/" .. action )
  defines { "TIXML_USE_STL" }

  if (_OPTIONS["with-clang"]) then
    buildoptions { "--stdlib=libc++" }
    kind "StaticLib"
  end

  if (_OPTIONS["with-boost"]) then
    boostlibs = "-L" .. _OPTIONS["with-boost"] .. "/lib " .. boostlibs
    boostinc = _OPTIONS["with-boost"] .. "/include"
  end

  if (_OPTIONS["with-bullet"]) then
    pkgconfigpaths = pkgconfigpaths .. _OPTIONS["with-bullet"] .. "/lib/pkgconfig:"
  end

  if (_OPTIONS["with-polyvox"]) then
    polyvoxlibs = "-L" .. _OPTIONS["with-polyvox"] .. "/lib " .. polyvoxlibs
    polyvoxinc = _OPTIONS["with-polyvox"] .. "/include/PolyVoxCore"
  end

  if (_OPTIONS["with-ogre"]) then
    pkgconfigpaths = pkgconfigpaths .. _OPTIONS["with-ogre"] .. "/lib/pkgconfig:"
  end

  pkgconfigenv = "PKG_CONFIG_PATH=" .. pkgconfigpaths .. "$$PKG_CONFIG_PATH"
  pkgconfig = pkgconfigenv .. " " .. pkgconfig

  configuration "Debug"
    targetdir ( "Build/" .. action .. "/bin/Debug" )
    flags { "Symbols" }

  configuration "Release"
    targetdir ( "Build/" .. action .. "/bin/Release" )
    defines { "NDEBUG" }
    flags { "Optimize" }
    buildoptions { "-msse2" }

  project "Game"
    kind "WindowedApp"
    files { "Game/**.h", "Game/**.cxx" }
    includedirs { ".", boostinc, polyvoxinc}
    links { }
    buildoptions { "`" .. pkgconfig .. " --cflags OGRE OIS bullet`" }
    linkoptions { "`" .. pkgconfig .. " --libs OGRE OIS bullet` " .. libdir .. " " .. polyvoxlibs .. " " .. boostlibs }
