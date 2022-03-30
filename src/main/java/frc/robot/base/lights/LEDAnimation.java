package frc.robot.base.lights;

public class LEDAnimation {
    
    private int default_color;

    private int[][] animation;


    public LEDAnimation(int default_color, int n_leds, int animation_length){
        animation = new int[animation_length][n_leds];

        this.default_color = default_color;

        for (int i = 0; i < animation_length; i++) {
            for (int j = 0; j < n_leds; j++ ) {
                animation[i][j] = default_color;
            }
        }
    }

    public void setLedColor(int color, int position, int frame){
        animation[frame][position] = color;
    }

    public void setLedDefault(int position, int frame){
        setLedColor(default_color, position, frame);
    }

    public void setFrameColor(int color, int frame){
        for(int i = 0; i < animation[frame].length; i++){
            animation[frame][i] = color;
        }
    }

    public void setFrameDefualt(int frame){
        setFrameColor(default_color, frame);
    }

}
