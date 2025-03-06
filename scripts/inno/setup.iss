; Script generated by the Inno Setup Script Wizard.
; SEE THE DOCUMENTATION FOR DETAILS ON CREATING INNO SETUP SCRIPT FILES!

#define AppName "BopenStarbound"
#define AppVersion "1.0"
#define AppExeName "starbound.exe"
#define AppServerExeName "starbound_server.exe"

[Setup]
SourceDir={#SourcePath}\..\..\
; NOTE: The value of AppId uniquely identifies this application. Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{08791089-2868-4FE5-ACC8-4473ACA67ED7}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
DefaultDirName={autopf}\{#AppName}
DisableProgramGroupPage=yes
; Uncomment the following line to run in non administrative install mode (install for current user only.)
;PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
OutputBaseFilename=BopenStarbound
SetupIconFile=scripts\inno\starbound.ico
Compression=lzma2/ultra64
SolidCompression=yes
ArchitecturesInstallIn64BitMode=x64
WizardStyle=modern
WizardImageAlphaFormat=premultiplied
WizardSmallImageFile=scripts\inno\small.bmp

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "windows\win\{#AppExeName}"; DestDir: "{app}\win\"; Flags: ignoreversion
Source: "windows\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Dirs]
Name: "{app}"; Permissions: users-modify

[Icons]
; Client
Name: "{autoprograms}\{#AppName}"; Filename: "{app}\win\{#AppExeName}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\win\{#AppExeName}"; Tasks: desktopicon
; Server
Name: "{autoprograms}\{#AppName} Server"; Filename: "{app}\win\{#AppServerExeName}"
Name: "{autodesktop}\{#AppName} Server"; Filename: "{app}\win\{#AppServerExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\win\{#AppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(AppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent
