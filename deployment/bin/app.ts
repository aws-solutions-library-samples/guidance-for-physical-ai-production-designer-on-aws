#!/usr/bin/env node
import { App } from 'aws-cdk-lib';
import { buildApp } from '../lib/app';

const app = new App();
buildApp(app);
app.synth();
