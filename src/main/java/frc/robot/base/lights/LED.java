package frc.robot.base.lights;

import java.awt.Color;
import java.util.function.Function;

public class LED {

    public enum ColorMode{
        DEFAULT,
        RGB,
        HSL;
    }

    private byte red;
    public byte getRed(){return red;}

    private byte green;
    public byte getGreen(){return green;}

    private byte blue;
    public byte getBlue(){return blue;}

    private ColorMode colorMode;

    public LED(byte a, byte b, byte c, ColorMode colorMode){
        
        if(colorMode == ColorMode.RGB || colorMode == ColorMode.DEFAULT){
            red   = a;
            blue  = b;
            green = c;
        }
        else if(colorMode == ColorMode.HSL){
            byte[] rgb = HSL_to_RGB(a, b, c);
            red   = rgb[0];
            blue  = rgb[1];
            green = rgb[2];
        }
    }

    public LED(byte[] abc, ColorMode colorMode){  
        this(abc[0], abc[1], abc[2], colorMode);
    }

    public static byte[] HSL_to_RGB(float h, float s, float b){
        int rgb = Color.HSBtoRGB(h/360, s, b);
        Color color = new Color(rgb);

        return new byte[]{
            (byte)color.getRed(),
            (byte)color.getGreen(),
            (byte)color.getBlue()
        };

    } public static byte[] HSL_to_RGB(float[] hsl){
        int rgb = Color.HSBtoRGB(hsl[0]/360, hsl[1], hsl[2]);
        Color color = new Color(rgb);

        return new byte[]{
            (byte)color.getRed(),
            (byte)color.getGreen(),
            (byte)color.getBlue()
        };
    }   
}
