# ST GDB Server
clion config 
``` 
<component name="ProjectRunConfigurationManager">
  <configuration default="false" name="Nucleo" type="com.jetbrains.cidr.embedded.customgdbserver.type" factoryName="com.jetbrains.cidr.embedded.customgdbserver.factory" PROGRAM_PARAMS="-p 2331 --verify --swd -cp &quot;$ST_CUBE_PROG_BIN$&quot;" REDIRECT_INPUT="false" ELEVATE="false" USE_EXTERNAL_CONSOLE="false" PASS_PARENT_ENVS_2="true" PROJECT_NAME="Nucleo476" TARGET_NAME="Nucleo476.elf" CONFIG_NAME="Debug" version="1" RUN_TARGET_PROJECT_NAME="Nucleo476" RUN_TARGET_NAME="Nucleo476.elf">
    <custom-gdb-server version="1" gdb-connect="localhost:2331" executable="C:\ST\STM32CubeIDE_1.5.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.win32_1.6.0.202101291314\tools\bin\ST-LINK_gdbserver.exe" warmup-ms="0" download-type="UPDATED_ONLY" reset-cmd="monitor reset" reset-after-download="true">
      <debugger kind="GDB" isBundled="true" />
    </custom-gdb-server>
    <method v="2">
      <option name="CLION.COMPOUND.BUILD" enabled="true" />
    </method>
  </configuration>
</component>
```
