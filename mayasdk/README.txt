Maya SDK
========

the Maya SDK consists of two parts:

* libraries, which Autosdesk distributes with the Maya runtime,
* C++ headers, which Autodesk distributes in a "devkit" archive.

maya2017-libs-win64.zip
-----------------------

The Maya2017.zip archive contains libraries (lib and dll) that were copied
from real Maya2017 install on Windows 10 (64 bits) with this command:

   cd /c/Program\ Files/Autodesk
   tar.exe czf ~/Maya2017.tar.gz  Maya2017/{lib,bin}/{Foundation,OpenMaya,OpenMayaUI,OpenMayaAnim,OpenMayaFX,OpenMayaRender}.*

then converted to a zip file

maya2017-headers-win64.zip
--------------------------

The include.zip archive contains a subset of the C++ headers,
that were extracted from the (autodesk-provided)
Maya2017_Update3_DEVKIT_Windows.zip archive.
The official autodesk archive could have been used instead, but
* it weights 151 MiB, where this minimalistic one is only 1.1 MiB.
* downloading it from autodesk website requires authentication.

Here is former appveyor code which used the autodesk archive directly:

  # fetch archives if they are not already in appaveyor cache
  - ps: |
      function FetchIfNeeded ($dest, $md5, $url) {
          if (-Not (Test-Path $dest)) {
              $dest_dir = Split-Path -Path $dest
              if (-Not (Test-Path $dest_dir)) {
                  New-Item -Path $dest_dir -type directory -force
              }
              Write-Host "downloading to $dest"
              # cfr https://www.appveyor.com/docs/how-to/download-file/
              (new-object net.webclient).DownloadFile($url, $dest)
          }
          $real_md5 = Get-FileHash $dest -Algorithm MD5
          if ($real_md5.Hash -ne $md5) {
              throw "bad md5sum: ${real_md5.Hash} instead of $md5"
          }
      }
      function FetchPkgIfNeeded ($relpath, $md5) {
          FetchIfNeeded -dest "${env:qitoolchain_pkgs_dir}/${relpath}" `
                        -md5 $md5 `
                        -url "${env:qitoolchain_pkgs_url}/${relpath}"
      }
      # fetch the Maya SDK archive (aka. "devkit")
      #
      # The url is short-lived, but it is ok because the downloaded archive
      # will be stored in appveyor's cache.
      #
      # Here is how to get a new url:
      #
      # - go to "Maya 2017 Update 3 - Developer Kit (aka devkit)" page:
      #   https://apps.autodesk.com/MAYA/en/Detail/Index?id=8656206734503135164&os=Win64&appLang=en
      # - sign in (important)
      # - then right-click on the "Download" button and choose "inspect"
      # - copy the "data-href" part, it should look like
      #   "/MAYA/en/Detail/Download?appId=8656206734503135164&appLang=en&os=Win64"
      #   (basically it is the same id)
      # - paste it in your browser, with a "apps.autodesk.com" prefix
      # - you should get a json page with a "DownloadUrl" field
      # - you're done!
      FetchIfNeeded -dest $env:maya_sdk_archive_path `
                    -md5 '31C3310E5002413956BB78FF4035CD66' `
                    -url 'https://autodesk-exchange-apps-v-1-5-staging.s3.amazonaws.com/data/content/files/assets/QP9QKVEMEAC3/332017/Maya2017_Update3_DEVKIT_Windows.zip?AWSAccessKeyId=AKIAI77WF7CFWDZD5B2A&Expires=1505928896&response-content-disposition=attachment%3B%20filename%3DMaya2017_Update3_DEVKIT_Windows.zip&response-content-type=application%2Foctet-stream&Signature=bOAdL7mSY335qeT6Frj9AQhX6f4%3D'

  # extract the C++ headers in MAYA_LOCATION\include, as recommanded in
  # http://help.autodesk.com/view/MAYAUL/2017/ENU/?guid=__files_Setting_up_your_build_env_Windows_env_32bit_and_64bit_htm
  - ps: |
      $tmp_dir = "toto"
      New-Item -Path $tmp_dir -type directory -force
      Push-Location $tmp_dir
      & 7z x $env:maya_sdk_archive_path devkitBase\include\maya -r
      Pop-Location
      New-Item -Path $env:MAYA_LOCATION -type directory -force
      Move-Item "$tmp_dir\devkitBase\include" $env:MAYA_LOCATION
      Remove-Item -Recurse -Force $tmp_dir

