package frc.robot;

import java.util.Date;

public class Timer {
	private long targetMillisec;
	private long startMillisec;
	private long pauseStartMillisec;
	private long totalPauseMillisec;

	Timer(long millisecIn){
		Date date = new Date();
		targetMillisec = millisecIn;
		startMillisec = date.getTime();
		pauseStartMillisec = 0;
		totalPauseMillisec = 0;
	}

	public boolean timerElasped() {
		Date date = new Date();
		if (pauseStartMillisec > 0) {
			return (false);
		}else{
			return (targetMillisec <= date.getTime() - startMillisec - totalPauseMillisec);
		}
	}

	public void reset() {
		Date date = new Date();
		startMillisec = date.getTime();
	}

	public void reset(long millisecIn) {
		Date date = new Date();
		startMillisec = date.getTime();
		targetMillisec = millisecIn;
	}

	public void pause() {
		Date date = new Date();
		pauseStartMillisec = date.getTime();
	}

	public void resume() {
		Date date = new Date();
		totalPauseMillisec += (date.getTime() - pauseStartMillisec);
		pauseStartMillisec = 0;
	}

}
