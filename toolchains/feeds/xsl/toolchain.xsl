<html xsl:version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns="http://www.w3.org/1999/xhtml">
  <body >
    <h2> Packages </h2>
    <ul>
      <xsl:for-each select="toolchain/package">
      <li>
        <a>
        <xsl:attribute name="href">
          <xsl:value-of select="@url" />
        </xsl:attribute>
        <xsl:value-of select="@name" />
        <xsl:choose>
          <xsl:when test="@version">
            -
            <xsl:value-of select="@version" />
          </xsl:when>
        </xsl:choose>
        </a>
      </li>
      </xsl:for-each>
    </ul>
    <h2> Feeds </h2>
    <ul>
      <xsl:for-each select="toolchain/feed">
      <li>
        <a>
        <xsl:attribute name="href">
          <xsl:value-of select="@url" />
        </xsl:attribute>
        <xsl:value-of select="@url" />
        </a>
      </li>
      </xsl:for-each>
    </ul>
  </body>
</html>


